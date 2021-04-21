from cereal import car
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.chrysler.values import DBC, STEER_THRESHOLD


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["SBW_ROT1"]['DrvRqShftROT']

  def update(self, cp, cp_cam):

    ret = car.CarState.new_message()

    #self.frame = int(cp.vl["EPS_STATUS"]['COUNTER'])

    ret.doorOpen = any([cp.vl["CBC_PT1"]['DRV_AJAR'],
                        cp.vl["CBC_PT1"]['PSG_AJAR'],
                        cp.vl["CBC_PT1"]['L_R_AJAR'],
                        cp.vl["CBC_PT1"]['R_R_AJAR']])
    ret.seatbeltUnlatched = cp.vl["ORC_A1"]['DrvSbltUnFltr'] == 1

    ret.brakePressed = cp.vl["ESP_A1"]['BrkPdl_Stat'] == 5  # human-only
    ret.brake = 0
    ret.brakeLights = ret.brakePressed
    ret.gas = cp.vl["ECM_CRUISE_MAP"]['Rel_Pdl_ENG']
    ret.gasPressed = ret.gas > 1e-5

    ret.espDisabled = (cp.vl["GW_I_C1"]['TRAC_PSD'] == 1)

    ret.wheelSpeeds.fl = cp.vl['ESP_A6']['WhlRPM_FL']
    ret.wheelSpeeds.rr = cp.vl['ESP_A6']['WhlRPM_RR']
    ret.wheelSpeeds.rl = cp.vl['ESP_A6']['WhlRPM_RL']
    ret.wheelSpeeds.fr = cp.vl['ESP_A6']['WhlRPM_FR']
    #ret.vEgoRaw = (cp.vl['SPEED_1']['SPEED_LEFT'] + cp.vl['SPEED_1']['SPEED_RIGHT']) / 2.
    #ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    #ret.standstill = not ret.vEgoRaw > 0.001

    ret.leftBlinker = cp.vl["StW_Actn_Rq"]['TurnIndLvr_Stat'] == 1
    ret.rightBlinker = cp.vl["StW_Actn_Rq"]['TurnIndLvr_Stat'] == 2
    ret.steeringAngleDeg = cp.vl["SCCM_STW_ANGL_STAT"]['LRW']
    ret.steeringRateDeg = cp.vl["SCCM_STW_ANGL_STAT"]['VLRW']
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl['SBW_ROT1']['DrvRqShftROT'], None))

    ret.cruiseState.enabled = cp.vl["DAS_A3"]['ACC_Engd'] == 7  # ACC is green.
    ret.cruiseState.available = ret.cruiseState.enabled  # FIXME: for now same as enabled
    ret.cruiseState.speed = cp.vl["DAS_A4"]['SetSpeed_KPH'] * CV.KPH_TO_MS
    # CRUISE_STATE is a three bit msg, 0 is off, 1 and 2 are Non-ACC mode, 3 and 4 are ACC mode, find if there are other states too
    ret.cruiseState.nonAdaptive = cp.vl["ECM_CRUISE_MAP"]['CRUISE_EGD']

    #ret.steeringTorque = cp.vl["EPS_STATUS"]["TORQUE_DRIVER"]
    #ret.steeringTorqueEps = cp.vl["EPS_STATUS"]["TORQUE_MOTOR"]
    #ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    #steer_state = cp.vl["EPS_STATUS"]["LKAS_STATE"]
    #ret.steerError = steer_state == 4 or (steer_state == 0 and ret.vEgo > self.CP.minSteerSpeed)

    ret.genericToggle = bool(cp.vl["StW_Actn_Rq"]['HiBmLvr_Stat'])

    #self.lkas_counter = cp_cam.vl["LKAS_COMMAND"]['COUNTER']
    #self.lkas_car_model = cp_cam.vl["LKAS_HUD"]['CAR_MODEL']
    #self.lkas_status_ok = cp_cam.vl["LKAS_HEARTBIT"]['LKAS_STATUS_OK']

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("DrvRqShftROT", "SBW_ROT1", 0),
      ("DRV_AJAR", "CBC_PT1", 0),
      ("PSG_AJAR", "CBC_PT1", 0),
      ("L_R_AJAR", "CBC_PT1", 0),
      ("R_R_AJAR", "CBC_PT1", 0),
      ("BrkPdl_Stat", "ESP_A1", 0),
      ("Rel_Pdl_ENG", "ECM_CRUISE_MAP", 0),
      #("SPEED_LEFT", "SPEED_1", 0),
      #("SPEED_RIGHT", "SPEED_1", 0),
      ("WhlRPM_FL", "ESP_A6", 0),
      ("WhlRPM_RR", "ESP_A6", 0),
      ("WhlRPM_RL", "ESP_A6", 0),
      ("WhlRPM_FR", "ESP_A6", 0),
      ("LRW", "SCCM_STW_ANGL_STAT", 0),
      ("VLRW", "SCCM_STW_ANGL_STAT", 0),
      ("TurnIndLvr_Stat", "StW_Actn_Rq", 0),
      ("ACC_Engd", "DAS_A3", 0),
      ("HiBmLvr_Stat", "StW_Actn_Rq", 0),
      ("SetSpeed_KPH", "DAS_A4", 0),
      ("CRUISE_EGD", "ECM_CRUISE_MAP", 0),
      #("TORQUE_DRIVER", "EPS_STATUS", 0),
      #("TORQUE_MOTOR", "EPS_STATUS", 0),
      #("LKAS_STATE", "EPS_STATUS", 1),
      #("COUNTER", "EPS_STATUS", -1),
      ("TRAC_PSD", "GW_I_C1", 0),
      ("DrvSbltUnFltr", "ORC_A1", 0),
    ]

    checks = [
      # sig_address, frequency
      ("ESP_A1", 50),
      #("EPS_STATUS", 100),
      #("SPEED_1", 100),
      ("ESP_A6", 50),
      ("SCCM_STW_ANGL_STAT", 100),
      ("ACC_2", 50),
      ("GEAR", 50),
      ("ECM_CRUISE_MAP", 50),
      ("DAS_A4", 15),
      ("StW_Actn_Rq", 10),
      ("ORC_A1", 2),
      ("CBC_PT1", 1),
      ("GW_I_C1", 1),
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      #("COUNTER", "LKAS_COMMAND", -1),
      #("CAR_MODEL", "LKAS_HUD", -1),
      #("LKAS_STATUS_OK", "LKAS_HEARTBIT", -1)
    ]
    checks = [
      #("LKAS_COMMAND", 100),
      #("LKAS_HEARTBIT", 10),
      #("LKAS_HUD", 4),
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)
