#include "utils.h"
#include "step.h"
#include "comm.h"

static constexpr double ARM_SPEED_RAD_S = 0.55;
static constexpr double HARVEST_APPROACH_SPEED_RAD_S = 0.35;
static constexpr double HARVEST_GRIP_SETTLE_SECONDS = 1.0;
static constexpr double HARVEST_RELEASE_SETTLE_SECONDS = 0.6;
// Keep the arm slow only while it carries a loaded tray. After releasing the
// tray, use the regular arm speed for the exit route and home return.
static constexpr double NEW_TRAY_LOADED_MOVE_SPEED_RAD_S = 0.15;
static constexpr double NEW_TRAY_EMPTY_MOVE_SPEED_RAD_S = ARM_SPEED_RAD_S;
static constexpr double FIRST_HOME_MOVE_TIME = 4.0;
static constexpr double RELEASE_POSE_SPEED_RAD_S = 0.65;

// true: 통신만 검증, false: 실제 전체 수확/트레이 시퀀스 수행
static constexpr bool COMM_TEST_ONLY = false;

static const JointPose HOME_LIFT_POSE = {
  1.1842331682475082,
  -0.5737088146694367,
  0.6550097964269619,
  -0.2807184841832795
};

static const JointPose SAFE_TRANSFER_POSE = {
  0.019941750242306266,
  -0.3896311201231599,
  0.26231071472823775,
  0.31753402309212087
};

// Harvest poses mapped directly to their taught slot numbers. The order in
// each row is joint1, joint2, joint3, joint4; gripper_left_joint from
// /joint_states is intentionally excluded.
// Each slot currently has one final taught pose, kept as a one-pose path so
// the common smooth-path motion code can be reused.
static const JointPose HARVEST_SLOT_1_PATH[] = {
  {1.5063691337034930, 0.6335340653965629, 1.0615147052166565, -1.3222914391576297}
};

static const JointPose HARVEST_SLOT_2_PATH[] = {
  {1.5125050568550353, 0.5967185264873076, 0.9572040116404334, -1.1642914180054087}
};

static const JointPose HARVEST_SLOT_3_PATH[] = {
  {1.5309128263096632, 0.6273981422450201, 0.7838641826093555, -1.1121360712172970}
};

static const JointPose HARVEST_SLOT_4_PATH[] = {
  {1.5232429223702350, 0.6043884304267357, 0.6611457195785042, -0.9633399347923897}
};

static const JointPose HARVEST_SLOT_5_PATH[] = {
  {1.5109710760671495, 0.7040971816393022, 0.47706802503222745, -0.9096506072163923}
};

static const JointPose *const HARVEST_PATHS[5] = {
  HARVEST_SLOT_1_PATH,
  HARVEST_SLOT_2_PATH,
  HARVEST_SLOT_3_PATH,
  HARVEST_SLOT_4_PATH,
  HARVEST_SLOT_5_PATH
};

static const uint8_t HARVEST_PATH_COUNTS[5] = {
  1,
  1,
  1,
  1,
  1
};

// Physical harvest order: slot 5 -> 4 -> 3 -> 2 -> 1. The tray-transfer
// route starts only after slot 1 has finished.
static const uint8_t HARVEST_ORDER[] = {
  4,  // slot 5
  3,  // slot 4
  2,  // slot 3
  1,  // slot 2
  0   // slot 1
};
static constexpr uint8_t HARVEST_ORDER_COUNT =
  sizeof(HARVEST_ORDER) / sizeof(HARVEST_ORDER[0]);

static const JointPose PLANT_DROP_POSE_A = {
  -2.9835926324377790,
  0.3328738309709771,
  0.7930680673366695,
  -0.7117670855791443
};

static const JointPose PLANT_DROP_POSE_B = {
  -2.9437091319527524,
  0.19328157927338374,
  0.8897088569734652,
  -0.2730485802438509
};

// New-tray route executed only after the final harvest (slot 1). Each pose
// contains joint1 through joint4; gripper_left_joint is operated separately.
static const JointPose FINAL_HARVEST_TRAY_PICKUP_APPROACH_POSE = {
  -2.9636508821952656,
  0.5184855063051397,
  -0.02147573103060596,
  -0.31600004230464895
};

static const JointPose FINAL_HARVEST_TRAY_PICKUP_POSE_1 = {
  -2.9575149590437233,
  0.8206797215186112,
  -0.30219421521367806,
  -0.08130098175814604
};

static const JointPose FINAL_HARVEST_TRAY_PICKUP_POSE_2 = {
  -3.000466421104521,
  1.0431069357620286,
  -0.7317088358216579,
  0.4494563708502861
};

// Smooth, payload-carrying route from pickup position 2 to the release pose.
// Intermediate joint positions limit the largest single-joint change to about
// 0.25 rad while the tray is held.
static const JointPose FINAL_HARVEST_TRAY_CARRY_PATH_1[] = {
  {-2.9887059017307309, 0.80764088482158281, -0.518996833234849, 0.36994503334488038},
  {-2.976945382356941, 0.572174833881137, -0.30628483064804007, 0.29043369583947465},
  {-2.9651848629831514, 0.33670878294069118, -0.093572828061231128, 0.21092235833406892},
  {-2.9534243436093615, 0.10124273200024536, 0.11913917452557776, 0.13141102082866318},
  {-2.9416638242355715, -0.13422331894020045, 0.33185117711238665, 0.051899683323257451},
  {-2.9299033048617815, -0.36968936988064627, 0.54456317969919565, -0.027611654182148282}
};

static const JointPose FINAL_HARVEST_TRAY_CARRY_PATH_2[] = {
  {-2.6780236594909592, -0.44362724385673424, 0.62770493840259733, -0.0828349625460314},
  {-2.4261440141201369, -0.51756511783282222, 0.710846697105999, -0.13805827090991452},
  {-2.1742643687493146, -0.59150299180891008, 0.79398845580940081, -0.19328157927379763},
  {-1.9223847233784925, -0.66544086578499806, 0.87713021451280249, -0.24850488763768075},
  {-1.6705050780076702, -0.739378739761086, 0.96027197321620417, -0.30372819600156387}
};

static const JointPose FINAL_HARVEST_TRAY_CARRY_PATH_3[] = {
  {-1.6515859816237473, -0.49931074645698315, 0.83937156148951364, -0.37199034106247492},
  {-1.6326668852398245, -0.25924275315288031, 0.71847114976282322, -0.44025248612338597},
  {-1.6137477888559015, -0.019174759848777367, 0.59757073803613259, -0.508514631184297},
  {-1.5948286924719786, 0.22089323345532541, 0.47667032630944223, -0.57677677624520807},
  {-1.5759095960880558, 0.46096122675942808, 0.35576991458275176, -0.645038921306119},
  {-1.5569904997041328, 0.7010292200635313, 0.23486950285606112, -0.71330106636703006}
};

static constexpr uint8_t FINAL_HARVEST_TRAY_CARRY_PATH_1_COUNT =
  sizeof(FINAL_HARVEST_TRAY_CARRY_PATH_1) /
  sizeof(FINAL_HARVEST_TRAY_CARRY_PATH_1[0]);
static constexpr uint8_t FINAL_HARVEST_TRAY_CARRY_PATH_2_COUNT =
  sizeof(FINAL_HARVEST_TRAY_CARRY_PATH_2) /
  sizeof(FINAL_HARVEST_TRAY_CARRY_PATH_2[0]);
static constexpr uint8_t FINAL_HARVEST_TRAY_CARRY_PATH_3_COUNT =
  sizeof(FINAL_HARVEST_TRAY_CARRY_PATH_3) /
  sizeof(FINAL_HARVEST_TRAY_CARRY_PATH_3[0]);

static const JointPose FINAL_HARVEST_TRAY_EXIT_POSE_1 = {
  -1.624485654371101,
  -0.1165825398795155,
  0.2316310989705248,
  -0.1641359443039705
};

static const JointPose FINAL_HARVEST_TRAY_EXIT_POSE_2 = {
  -0.11198059751585854,
  0.018407769454420908,
  -0.030679615757919887,
  0.12271846303064438
};

static const JointPose FINAL_HARVEST_TRAY_EXIT_POSE_3 = {
  1.4158642672182395,
  -0.7777282594582271,
  0.4862719097595414,
  0.1764077906066417
};

static bool sequence_failed = false;
static bool manipulator_ready = false;
static bool estopped = false;
static bool job_running = false;

static uint8_t active_job_id = 0;
static uint8_t last_completed_job_id = 0;
static bool last_completed_valid = false;

static void enterEstop()
{
  const uint8_t stopped_job_id = active_job_id;

  stopRail();

  estopped = true;
  sequence_failed = true;
  job_running = false;

  commSendState(
    MANIP_STATE_ESTOP,
    stopped_job_id
  );

  active_job_id = 0;
}

static bool checkMove(bool result, const char *move_name)
{
  if (result)
  {
    return true;
  }

  Serial.print("[SYSTEM] MOVE FAILED: ");
  Serial.println(move_name);
  return false;
}

static bool moveHome()
{
  Serial.println("[ARM] MOVE HOME");

  return checkMove(
    movePoseAtSpeed(HOME_LIFT_POSE, ARM_SPEED_RAD_S),
    "HOME"
  );
}

// Tray-transfer route that was used when the slot 1 through 5 harvest poses
// above were tuned.
static bool transferNewTrayAfterFinalHarvest()
{
  Serial.println("[NEW TRAY] FINAL-HARVEST SEQUENCE START");

  Serial.println("[NEW TRAY] MOVE TO PICKUP APPROACH");
  if (!checkMove(
        movePoseAtSpeed(
          FINAL_HARVEST_TRAY_PICKUP_APPROACH_POSE,
          NEW_TRAY_LOADED_MOVE_SPEED_RAD_S
        ),
        "NEW TRAY PICKUP APPROACH"
      ))
  {
    return false;
  }

  Serial.println("[NEW TRAY] MOVE TO PICKUP POSITION 1");
  if (!checkMove(
        movePoseAtSpeed(
          FINAL_HARVEST_TRAY_PICKUP_POSE_1,
          NEW_TRAY_LOADED_MOVE_SPEED_RAD_S
        ),
        "NEW TRAY PICKUP POSITION 1"
      ))
  {
    return false;
  }

  Serial.println("[NEW TRAY] MOVE TO PICKUP POSITION 2");
  if (!checkMove(
        movePoseAtSpeed(
          FINAL_HARVEST_TRAY_PICKUP_POSE_2,
          NEW_TRAY_LOADED_MOVE_SPEED_RAD_S
        ),
        "NEW TRAY PICKUP POSITION 2"
      ))
  {
    return false;
  }

  Serial.println("[NEW TRAY] CLOSE GRIPPER");
  if (!closeGripper())
  {
    Serial.println("[SYSTEM] NEW TRAY GRIP FAILED");
    return false;
  }

  Serial.println("[NEW TRAY] FOLLOW CARRY PATH 1");
  if (!checkMove(
        movePosePathAtSpeed(
          FINAL_HARVEST_TRAY_CARRY_PATH_1,
          FINAL_HARVEST_TRAY_CARRY_PATH_1_COUNT,
          NEW_TRAY_LOADED_MOVE_SPEED_RAD_S
        ),
        "NEW TRAY CARRY PATH 1"
      ))
  {
    return false;
  }

  Serial.println("[NEW TRAY] FOLLOW CARRY PATH 2");
  if (!checkMove(
        movePosePathAtSpeed(
          FINAL_HARVEST_TRAY_CARRY_PATH_2,
          FINAL_HARVEST_TRAY_CARRY_PATH_2_COUNT,
          NEW_TRAY_LOADED_MOVE_SPEED_RAD_S
        ),
        "NEW TRAY CARRY PATH 2"
      ))
  {
    return false;
  }

  Serial.println("[NEW TRAY] FOLLOW CARRY PATH 3");
  if (!checkMove(
        movePosePathAtSpeed(
          FINAL_HARVEST_TRAY_CARRY_PATH_3,
          FINAL_HARVEST_TRAY_CARRY_PATH_3_COUNT,
          NEW_TRAY_LOADED_MOVE_SPEED_RAD_S
        ),
        "NEW TRAY CARRY PATH 3"
      ))
  {
    return false;
  }

  Serial.println("[NEW TRAY] OPEN GRIPPER");
  if (!openGripper())
  {
    Serial.println("[SYSTEM] NEW TRAY RELEASE FAILED");
    return false;
  }

  Serial.println("[NEW TRAY] EXIT POSE 1");
  if (!checkMove(
        movePoseAtSpeed(
          FINAL_HARVEST_TRAY_EXIT_POSE_1,
          NEW_TRAY_EMPTY_MOVE_SPEED_RAD_S
        ),
        "NEW TRAY EXIT POSE 1"
      ))
  {
    return false;
  }

  Serial.println("[NEW TRAY] EXIT POSE 2");
  if (!checkMove(
        movePoseAtSpeed(
          FINAL_HARVEST_TRAY_EXIT_POSE_2,
          NEW_TRAY_EMPTY_MOVE_SPEED_RAD_S
        ),
        "NEW TRAY EXIT POSE 2"
      ))
  {
    return false;
  }

  Serial.println("[NEW TRAY] EXIT POSE 3");
  if (!checkMove(
        movePoseAtSpeed(
          FINAL_HARVEST_TRAY_EXIT_POSE_3,
          NEW_TRAY_EMPTY_MOVE_SPEED_RAD_S
        ),
        "NEW TRAY EXIT POSE 3"
      ))
  {
    return false;
  }

  Serial.println("[NEW TRAY] RETURN HOME");
  if (!moveHome())
  {
    return false;
  }

  Serial.println("[NEW TRAY] FINAL-HARVEST SEQUENCE COMPLETE");
  return true;
}

static bool completePlantReleaseABABA()
{
  // The arm is already at pose A when this function is called.
  Serial.println("[PLANT RELEASE] A-B-A-B-A START");

  if (!checkMove(
        movePoseAtSpeed(PLANT_DROP_POSE_B, RELEASE_POSE_SPEED_RAD_S),
        "PLANT POSE B 1"
      ))
  {
    return false;
  }

  if (!checkMove(
        movePoseAtSpeed(PLANT_DROP_POSE_A, RELEASE_POSE_SPEED_RAD_S),
        "PLANT POSE A 2"
      ))
  {
    return false;
  }

  if (!checkMove(
        movePoseAtSpeed(PLANT_DROP_POSE_B, RELEASE_POSE_SPEED_RAD_S),
        "PLANT POSE B 2"
      ))
  {
    return false;
  }

  if (!checkMove(
        movePoseAtSpeed(PLANT_DROP_POSE_A, RELEASE_POSE_SPEED_RAD_S),
        "PLANT POSE A 3"
      ))
  {
    return false;
  }

  Serial.println("[PLANT RELEASE] A-B-A-B-A COMPLETE");
  return true;
}

static bool harvestSlot(uint8_t slot)
{
  if (slot >= 5)
  {
    Serial.println("[SYSTEM] INVALID HARVEST SLOT");
    return false;
  }

  Serial.print("[HARVEST] SLOT ");
  Serial.print(slot + 1);
  Serial.println(" START");

  // Always begin a pickup with a confirmed open gripper. This keeps the
  // gripper state independent of how the preceding slot ended.
  Serial.println("[HARVEST] OPEN GRIPPER FOR PICKUP");
  if (!openGripper())
  {
    Serial.println("[SYSTEM] GRIPPER OPEN FAILED BEFORE PICKUP");
    return false;
  }

  if (!runManipulator(0.3))
  {
    return false;
  }

  Serial.println("[HARVEST] FOLLOW SMOOTH APPROACH PATH");
  if (!checkMove(
        movePosePathAtSpeed(
          HARVEST_PATHS[slot],
          HARVEST_PATH_COUNTS[slot],
          HARVEST_APPROACH_SPEED_RAD_S
        ),
        "HARVEST APPROACH PATH"
      ))
  {
    return false;
  }

  // The arm has reached the taught harvest pose. Close and wait before
  // lifting so the crop is secured before any arm movement begins.
  Serial.println("[HARVEST] CLOSE GRIPPER AND SECURE CROP");
  if (!closeGripper())
  {
    Serial.println("[SYSTEM] GRIPPER CLOSE FAILED");
    return false;
  }

  Serial.println("[HARVEST] GRIP SETTLE BEFORE LIFT");
  if (!runManipulator(HARVEST_GRIP_SETTLE_SECONDS))
  {
    return false;
  }

  Serial.println("[HARVEST] LIFT");
  if (!checkMove(
        movePoseAtSpeed(HOME_LIFT_POSE, ARM_SPEED_RAD_S),
        "HARVEST LIFT"
      ))
  {
    return false;
  }

  Serial.println("[HARVEST] SAFE TRANSFER POSE");
  if (!checkMove(
        movePoseAtSpeed(SAFE_TRANSFER_POSE, ARM_SPEED_RAD_S),
        "SAFE TRANSFER POSE"
      ))
  {
    return false;
  }

  Serial.println("[HARVEST] MOVE TO PLANT POSE A");
  if (!checkMove(
        movePoseAtSpeed(PLANT_DROP_POSE_A, ARM_SPEED_RAD_S),
        "PLANT POSE A"
      ))
  {
    return false;
  }

  // Open only after the arm is confirmed at the plant release pose.
  Serial.println("[HARVEST] OPEN GRIPPER AND RELEASE CROP");
  if (!openGripper())
  {
    Serial.println("[SYSTEM] GRIPPER OPEN FAILED");
    return false;
  }

  if (!runManipulator(HARVEST_RELEASE_SETTLE_SECONDS))
  {
    return false;
  }

  if (!completePlantReleaseABABA())
  {
    return false;
  }

  if (slot == 0)
  {
    // Slot 1 is the final harvest. Continue with the tray-transfer route
    // that was used during the slot-pose tuning sequence.
    if (!transferNewTrayAfterFinalHarvest())
    {
      return false;
    }
  }
  else
  {
    Serial.println("[HARVEST] RETURN HOME");
    if (!moveHome())
    {
      return false;
    }
  }

  Serial.print("[HARVEST] SLOT ");
  Serial.print(slot + 1);
  Serial.println(" COMPLETE");
  return true;
}

// =====================================================
// 현재 활성화된 테스트 시퀀스
// 홈 → 그리퍼 열기 → 레일 호밍 → 5번 수확 → 홈 복귀
// =====================================================

// =====================================================
// 1번 칸 수확 후 새 트레이 이송 검증 시퀀스
// 현재 활성화됨
// =====================================================

static bool runFullHarvestAndTraySequence()
{
  Serial.println("[SYSTEM] FULL HARVEST + NEW TRAY SEQUENCE START");

  if (!checkMove(
        movePoseTimed(HOME_LIFT_POSE, FIRST_HOME_MOVE_TIME),
        "INITIAL HOME"
      ))
  {
    return false;
  }

  if (!openGripper())
  {
    return false;
  }

  if (!homeRail())
  {
    return false;
  }

  if (!runManipulator(0.5))
  {
    return false;
  }

  for (uint8_t order_index = 0;
       order_index < HARVEST_ORDER_COUNT;
       ++order_index)
  {
    const uint8_t slot = HARVEST_ORDER[order_index];
    if (!harvestSlot(slot))
    {
      Serial.println("[SYSTEM] HARVEST SEQUENCE ABORTED");
      return false;
    }
  }

  Serial.println("[SYSTEM] FULL HARVEST + NEW TRAY SEQUENCE COMPLETE");
  return true;
}

void setup()
{
  Serial.begin(115200);

  // 현재 통신 포트는 기존 comm 구조와 동일하게 Serial 사용.
  // 이후 실제 연결 포트가 Serial1/Serial2 등으로 바뀌었다면
  // Serial.begin / commBegin 부분만 해당 포트로 변경하면 됨.
  commBegin(Serial);

  const unsigned long serial_start_ms = millis();

  while (!Serial && millis() - serial_start_ms < 3000)
  {
    delay(10);
  }

  delay(300);

  initializeRail();

  if (!initManipulator())
  {
    manipulator_ready = false;
    sequence_failed = true;

    commSendState(
      MANIP_STATE_ERROR,
      0
    );

    commSendError(
      0,
      MANIP_ERR_NOT_READY
    );

    return;
  }

  // Dynamixel/OpenManipulator 초기 처리
  if (!runManipulator(1.0))
  {
    commTakeEstop();
    enterEstop();
    return;
  }

  manipulator_ready = true;
  sequence_failed = false;
  estopped = false;
  job_running = false;
  active_job_id = 0;

  // Master에 준비 완료 보고
  commSendState(
    MANIP_STATE_IDLE,
    0
  );
}

void loop()
{
  processManipulatorOnce();
  commPoll();

  // ESTOP은 RESET보다 우선 처리
  if (commTakeEstop())
  {
    enterEstop();
    return;
  }

  if (commTakeReset())
  {
    stopRail();

    estopped = false;
    sequence_failed = false;
    job_running = false;
    active_job_id = 0;

    // RESET 이후 같은 job_id도 새 작업으로 받을 수 있게 초기화
    last_completed_valid = false;

    if (manipulator_ready)
    {
      commSendState(
        MANIP_STATE_IDLE,
        0
      );
    }
    else
    {
      commSendState(
        MANIP_STATE_ERROR,
        0
      );
    }

    return;
  }

  uint8_t requested_job_id = 0;

  if (!commTakeHarvestJob(requested_job_id))
  {
    delay(10);
    return;
  }

  // Master가 이미 완료된 동일 job_id를 재전송하면
  // 실제 로봇 동작은 반복하지 않고 DONE만 재전송
  if (
    last_completed_valid &&
    requested_job_id == last_completed_job_id
  )
  {
    commSendDone(requested_job_id);
    return;
  }

  if (!manipulator_ready)
  {
    commSendError(
      requested_job_id,
      MANIP_ERR_NOT_READY
    );
    return;
  }

  if (estopped)
  {
    commSendError(
      requested_job_id,
      MANIP_ERR_ESTOP
    );
    return;
  }

  // 이전 시퀀스 실패 후 RESET하지 않은 상태
  if (sequence_failed)
  {
    commSendError(
      requested_job_id,
      MANIP_ERR_SEQUENCE_FAILED
    );
    return;
  }

  if (job_running)
  {
    commSendError(
      requested_job_id,
      MANIP_ERR_BUSY
    );
    return;
  }

  job_running = true;
  active_job_id = requested_job_id;

  commSendState(
    MANIP_STATE_HARVESTING,
    active_job_id
  );

  bool result = false;

  if (COMM_TEST_ONLY)
  {
    result = runManipulator(3.0);
  }
  else
  {
    // 최신 동작:
    // HOME -> rail homing -> harvest 5,4,3,2,1
    // -> slot 1 완료 후 new-tray transfer
    result = runFullHarvestAndTraySequence();
  }

  // 시퀀스 종료 직후 들어온 ESTOP까지 확인
  commPoll();

  if (commTakeEstop())
  {
    enterEstop();
    return;
  }

  const uint8_t finished_job_id = active_job_id;

  if (result)
  {
    last_completed_job_id = finished_job_id;
    last_completed_valid = true;
    sequence_failed = false;

    commSendDone(
      finished_job_id
    );
  }
  else
  {
    sequence_failed = true;

    commSendError(
      finished_job_id,
      MANIP_ERR_SEQUENCE_FAILED
    );

    commSendState(
      MANIP_STATE_ERROR,
      finished_job_id
    );
  }

  job_running = false;
  active_job_id = 0;

  if (!sequence_failed)
  {
    commSendState(
      MANIP_STATE_IDLE,
      0
    );
  }

  delay(10);
}
