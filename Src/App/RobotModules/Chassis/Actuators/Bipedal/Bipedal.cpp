#include "Bipedal.hpp"
#include "M3508.hpp"
#include "DM8009.hpp"
#include "TopicRouter.hpp"
#include "dsp/fast_math_functions.h"
#include "sdkconfig.h"
#include "Bsp_can.hpp"
#include "Dwt.hpp"

// #include "OSnew.hpp"

#include "MotorCommonMacros.hpp"

#include "./State/StopState.hpp"
#include "./State/RunState.hpp"
#include "./State/RescueState.hpp"
#include "./State/LaunchState.hpp"
#include "./State/TestState.hpp"

extern canHandle HCAN1;
extern canHandle HCAN2;

#define HIP_HANDLE   HCAN2
#define WHEEL_HANDLE HCAN1

using namespace CHASSIS;
using namespace PINYMOTOR;

void ChassisCtrlRxPacket::registerCallback()
{
    Can::instance().registerCallback(
            &HCAN1, uid(), [this](const uint8_t *_rxBuf) {
                xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
            });
    LOG::info("ChassisCtrlRxPacket", " Receive cb registered, uId:0x%hx \n",
              uid());
}

Bipedal::Bipedal()
        : insSub_(new Subscriber<INS_SYS::INSData_s>(
                  2, &TopicRouter::instance().insTopic, &insData_))
{
    InitConfig_s rfDM8009Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HIP_HANDLE),
        .comType = ComType_e::FDCAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 1,
        .txFreq = CTRL_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr,
        .isReverse = true,
    };
    motors_.HipRF =
            new DMMOTOR::DM8009("HipRF", rfDM8009Config); // HIP1控制电机
    motors_.HipRF->setZeroAng(rangeMap(RF_HIP_ZERO_ANG - PI));

    InitConfig_s lfDM8009Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HIP_HANDLE),
        .comType = ComType_e::FDCAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 2,
        .txFreq = CTRL_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr,
        .isReverse = false,
    };
    motors_.HipLF =
            new DMMOTOR::DM8009("HipLF", lfDM8009Config); // HIP1控制电机
    motors_.HipLF->setZeroAng(rangeMap(LF_HIP_ZERO_ANG - PI));

    InitConfig_s lbDM8009Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HIP_HANDLE),
        .comType = ComType_e::FDCAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 3,
        .txFreq = CTRL_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr,
        .isReverse = false,
    };
    motors_.HipLB =
            new DMMOTOR::DM8009("HipLB", lbDM8009Config); // HIP4控制电机
    motors_.HipLB->setZeroAng(LB_HIP_ZERO_ANG);

    InitConfig_s rbDM8009Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HIP_HANDLE),
        .comType = ComType_e::FDCAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 4,
        .txFreq = CTRL_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr,
        .isReverse = true,
    };
    motors_.HipRB =
            new DMMOTOR::DM8009("HipRB", rbDM8009Config); // HIP4控制电机
    motors_.HipRB->setZeroAng(RB_HIP_ZERO_ANG);

    InitConfig_s lM3508Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&WHEEL_HANDLE),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 5,
        .txFreq = CTRL_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr,
        .isReverse = true,
    };
    motors_.WheelL = new DJIMOTOR::M3508("WheelL", lM3508Config);
    motors_.WheelL->overrideReductionRatio(DJIMOTOR::M3508::XROLL_GEARBOX_RR);

    InitConfig_s rM3508Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&WHEEL_HANDLE),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 6,
        .txFreq = CTRL_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr,
        .isReverse = false,
    };
    motors_.WheelR = new DJIMOTOR::M3508("WheelR", rM3508Config);
    motors_.WheelR->overrideReductionRatio(DJIMOTOR::M3508::XROLL_GEARBOX_RR);

    legL_.init(motors_.HipLF, motors_.HipLB, motors_.WheelL);
    legR_.init(motors_.HipRF, motors_.HipRB, motors_.WheelR);

    this->stateFactory_.addState(FSMState_e::STOP,
                                 std::make_unique<StopState>(this));
    this->stateFactory_.addState(FSMState_e::LAUNCH,
                                 std::make_unique<LaunchState>(this));
    this->stateFactory_.addState(FSMState_e::RESCUE,
                                 std::make_unique<RescueState>(this));
    this->stateFactory_.addState(FSMState_e::RUN,
                                 std::make_unique<RunState>(this));
    this->stateFactory_.addState(FSMState_e::TEST,
                                 std::make_unique<TestState>(this));
    this->stateFactory_.init(
            this->stateFactory_.getNextState(FSMState_e::STOP));
}

void Bipedal::stop()
{
    legL_.stop();
    legR_.stop();

    rollAngPid_.reset();
    rollVelPid_.reset();

    hipAngPidL_.reset();
    hipVelPidL_.reset();
    hipAngPidR_.reset();
    hipVelPidR_.reset();

    wzVelPid_.reset();
    dsPid_[0].reset();
    dsPid_[1].reset();

    twLPF_[0].reset();
    twLPF_[1].reset();

    tpLPF_[0].reset();
    tpLPF_[1].reset();
}

void Bipedal::enter()
{
    legL_.enter();
    legR_.enter();

    s_ = 0;
    setRoll_ = 0;
}

void Bipedal::update(void *_param)
{
    // #if !APP_USE_COMM
    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &msg, 0) ==
        pdTRUE) {
    };
    // #else
    //     msg.vx = chassisCtrlRxPacket_.data().vx;
    //     msg.vy = chassisCtrlRxPacket_.data().vy;
    // #endif

    // update dt
    dt_ = Dwt::instance().getDt(&dtCnt_);

    // update insData
    insSub_->receive();

    // update ax az
    float sinPitch = arm_sin_f32(insData_.pitch);
    float cosPitch = arm_cos_f32(insData_.pitch);
    float cosRoll = arm_cos_f32(insData_.roll);

    float axFromX =
            (insData_.body.ax - (WheelLegged::GRAVITY * -sinPitch)) * cosPitch;
    float axFromZ =
            (-insData_.body.az - (WheelLegged::GRAVITY * cosPitch * cosRoll)) *
            sinPitch * cosRoll;
    float axFromINS = axFromX + axFromZ;

    float azFromX =
            (insData_.body.ax - (WheelLegged::GRAVITY * -sinPitch)) * -sinPitch;
    float azFromZ = -insData_.body.az - (WheelLegged::GRAVITY * cosPitch);
    // (-insData_.body.az - (WheelLegged::GRAVITY * cosPitch * cosRoll)) *
    // cosPitch * cosRoll;
    float azFromINS = azFromX + azFromZ;

    // update leg
    legL_.stateUpdate(dt_);
    legR_.stateUpdate(dt_);

    legL_.isFloating = detector_.isFloating(legL_.calcFn(az_, insData_.pitch));
    legR_.isFloating = detector_.isFloating(legR_.calcFn(az_, insData_.pitch));

    // update odom
    float vxFromWheel = (wheelFltr[0].process(motors_.WheelL->vel()) +
                         wheelFltr[1].process(motors_.WheelR->vel())) *
                        W_RADIUS / 2.f;
    odom_.update(dt_, vxFromWheel, axFromINS, yaw_);
    // ds_ = odom_.normVel();
    ds_ = vxFromWheel;
    s_ = clamp((detector_.isStop(this->msg.vx, ds_)) ? s_ + (ds_ * dt_) : 0,
               -MAX_S, MAX_S);
    if (isFloating())
        s_ = 0;

    as_ = odom_.normAcc();

    // update other state
    yaw_ = insData_.yaw;

    wz_ = insData_.body.gz; // TODO: earth.gz
    wx_ = insData_.body.gx; // TODO: earth.gx
    az_ = azFromINS;

    // update state
    this->stateFactory_.update();
}

void Bipedal::ctrl()
{
    legL_.ctrl();
    legR_.ctrl();
}

void Bipedal::setYawZero(float _yawZero) { yawZero_ = _yawZero; }

void Bipedal::setRoll(float _roll)
{
    setRoll_ = _roll;
    float dRoll = rollAngPid_.calc(setRoll_, insData_.roll);
    float delta = rollVelPid_.calc(dRoll, wx_);
    setLength(legL_.state().L0 + delta, legR_.state().L0 - delta);
}

void Bipedal::setLengthSmooth(float _l, float _step)
{
    setLengthSmooth(_l, _l, _step);
}
void Bipedal::setLengthSmooth(float _lL, float _lR, float _step)
{
    float deltaL = _lL - legL_.state().L0;
    deltaL = clamp(deltaL, -_step, _step);
    float deltaR = _lR - legR_.state().L0;
    deltaR = clamp(deltaR, -_step, _step);
    setLength(legL_.state().L0 + deltaL, legR_.state().L0 + deltaR);
}
void Bipedal::setLengthSmoothL(float _l, float _step)
{
    float delta = _l - legL_.state().L0;
    delta = clamp(delta, -_step, _step);
    setLengthL(legL_.state().L0 + delta);
}
void Bipedal::setLengthSmoothR(float _l, float _step)
{
    float delta = _l - legR_.state().L0;
    delta = clamp(delta, -_step, _step);
    setLengthR(legR_.state().L0 + delta);
}

void Bipedal::setLength(float _l) { setLength(_l, _l); }

void Bipedal::setLength(float _lL, float _lR)
{
    setLengthL(_lL);
    setLengthR(_lR);
}

void Bipedal::setLengthL(float _l) { legL_.setL(_l); }

void Bipedal::setLengthR(float _l) { legR_.setL(_l); }

void Bipedal::setFdForward(float _fd) { setFdForward(_fd, _fd); }

void Bipedal::setFdForward(float _fdL, float _fdR)
{
    legL_.setFdForward(_fdL);
    legR_.setFdForward(_fdR);
}

void Bipedal::setHipPhiRot(float _dphi) { setHipPhiRot(_dphi, _dphi); }
void Bipedal::setHipPhiRot(float _dphiL, float _dphiR)
{
    setHipPhiRotL(_dphiL);
    setHipPhiRotR(_dphiR);
}

void Bipedal::setHipPhiRotL(float _dphi)
{
    if (_dphi == 0)
        hipAngPidL_.reset();
    setHipPhiL(legL_.state().phi0 + _dphi);
}
void Bipedal::setHipPhiRotR(float _dphi)
{
    if (_dphi == 0)
        hipAngPidR_.reset();
    setHipPhiR(legR_.state().phi0 + _dphi);
}

void Bipedal::setHipPhiSmooth(float _phi, float _step)
{
    setHipPhiSmooth(_phi, _phi, _step);
}
void Bipedal::setHipPhiSmooth(float _phiL, float _phiR, float _step)
{
    setHipPhiSmoothL(_phiL, _step);
    setHipPhiSmoothR(_phiR, _step);
}

void Bipedal::setHipPhiSmoothL(float _phi, float _step)
{
    float deltaL = getMinorArc(_phi, legL_.state().phi0);
    deltaL = clamp(deltaL, -_step, _step);
    setHipPhiL(legL_.state().phi0 + deltaL);
}
void Bipedal::setHipPhiSmoothR(float _phi, float _step)
{
    float deltaR = getMinorArc(_phi, legR_.state().phi0);
    deltaR = clamp(deltaR, -_step, _step);
    setHipPhiR(legR_.state().phi0 + deltaR);
}

void Bipedal::setHipPhi(float _phi) { setHipPhi(_phi, _phi); }

void Bipedal::setHipPhi(float _phiL, float _phiR)
{
    setHipPhiL(_phiL);
    setHipPhiR(_phiR);
}

void Bipedal::setHipPhiL(float _phi)
{
    float hipVelL = hipAngPidL_.calc(getMinorArc(_phi, legL_.state().phi0), 0);
    float tpL = hipVelPidL_.calc(hipVelL, legL_.state().dphi0);
    legL_.setTp(tpL);
}

void Bipedal::setHipPhiR(float _phi)
{
    float hipVelR = hipAngPidR_.calc(getMinorArc(_phi, legR_.state().phi0), 0);
    float tpR = hipVelPidR_.calc(hipVelR, legR_.state().dphi0);
    legR_.setTp(tpR);
}

void Bipedal::setHipTp(float _tp) { setHipTp(_tp, _tp); }

void Bipedal::setHipTp(float _tpL, float _tpR)
{
    legL_.setTp(_tpL);
    legR_.setTp(_tpR);
}

void Bipedal::setHipFd(float _fd) { setHipFd(_fd, _fd); }

void Bipedal::setHipFd(float _fdL, float _fdR)
{
    legL_.setFdForward(_fdL);
    legR_.setFdForward(_fdR);
}

bool Bipedal::isTumble()
{
    return detector_.detectTumble(legL_.state(), legR_.state(), insData_);
}

bool Bipedal::isFreeze() { return detector_.detectFreeze(insData_); }

bool Bipedal::isOverturn() { return insData_.body.az > 0.1f; }

bool Bipedal::isLaunched(float _setPhi, float _phiTh, float _setL, float _lTh)
{
    const auto &leftState = legL_.state();
    const auto &rightState = legR_.state();

    return fabs(getMinorArc(_setPhi, leftState.phi0)) < _phiTh &&
           fabs(getMinorArc(_setPhi, rightState.phi0)) < _phiTh &&
           fabs(leftState.L0 - _setL) < _lTh &&
           fabs(rightState.L0 - _setL) < _lTh;
}

/**
 * @brief detect if the leg is in the swing phase
 * 
 * @param _ang1 clockwise leading phase
 * @param _ang2 clockwise lag phase
 * @return true if the leg is in the swing phase
 * @return false if the leg is in the stance phase
 */
bool Bipedal::detectPhiL(float _ang1, float _ang2)
{
    return getMinorArc(_ang1, legL_.state().phi0) > 0 &&
           getMinorArc(_ang2, legL_.state().phi0) < 0;
}

bool Bipedal::detectPhiR(float _ang1, float _ang2)
{
    return getMinorArc(_ang1, legR_.state().phi0) > 0 &&
           getMinorArc(_ang2, legR_.state().phi0) < 0;
}

float Bipedal::yaw() const { return yaw_; }

bool Bipedal::isFloating() const
{
    return legL_.isFloating && legR_.isFloating;
}

float Bipedal::phiL() const { return legL_.state().phi0; }

float Bipedal::phiR() const { return legR_.state().phi0; }

float Bipedal::lengthL() const { return legL_.state().L0; }

float Bipedal::lengthR() const { return legR_.state().L0; }

float Bipedal::vel() const { return ds_; }

float Bipedal::calcFdForwardL()
{
    float ang = legL_.state().phi0 + insData_.pitch;
    bool flag = getMinorArc(PI / 2.5f, ang) > 0 &&
                getMinorArc(-PI / 2.5f, ang) < 0;
    return flag ? FdForward_ / 2.f * arm_cos_f32(ang) : 0.f;
}

float Bipedal::calcFdForwardR()
{
    float ang = legR_.state().phi0 + insData_.pitch;
    bool flag = getMinorArc(PI / 2.5f, ang) > 0 &&
                getMinorArc(-PI / 2.5f, ang) < 0;
    return flag ? FdForward_ / 2.f * arm_cos_f32(ang) : 0.f;
}
