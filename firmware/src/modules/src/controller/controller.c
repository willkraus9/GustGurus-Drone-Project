#define DEBUG_MODULE "CONTROLLER"
#include "debug.h"

#include "cfassert.h"
#include "controller.h"
#include "controller_pid.h"
#include "controller_mellinger.h"
#include "controller_indi.h"
#include "controller_brescianini.h"
#include "controller_lee.h"
#include "controller_custom1.h"
#include "controller_custom2.h"
#include "controller_custom3.h"
#include "controller_custom4.h"
#include "controller_custom5.h"

#include "autoconf.h"

#define DEFAULT_CONTROLLER ControllerTypePID
static ControllerType currentController = ControllerTypeAutoSelect;

static void initController();

typedef struct {
  void (*init)(void);
  bool (*test)(void);
  void (*update)(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick);
  const char* name;
} ControllerFcns;

static ControllerFcns controllerFunctions[] = {
  {.init = 0, .test = 0, .update = 0, .name = "None"}, // Any
  {.init = controllerPidInit, .test = controllerPidTest, .update = controllerPid, .name = "PID"},
  {.init = controllerMellingerFirmwareInit, .test = controllerMellingerFirmwareTest, .update = controllerMellingerFirmware, .name = "Mellinger"},
  {.init = controllerINDIInit, .test = controllerINDITest, .update = controllerINDI, .name = "INDI"},
  {.init = controllerBrescianiniInit, .test = controllerBrescianiniTest, .update = controllerBrescianini, .name = "Brescianini"},
  // {.init = controllerLeeFirmwareInit, .test = controllerLeeFirmwareTest, .update = controllerLeeFirmware, .name = "Lee"},
  {.init = controllerCustomFirmware1Init, .test = controllerCustomFirmware1Test, .update = controllerCustomFirmware1, .name = "Custom Controller 1 - SMC controller kaustabh"},
  {.init = controllerCustomFirmware2Init, .test = controllerCustomFirmware2Test, .update = controllerCustomFirmware2, .name = "Custom Controller 2 - PID Direct Thrust Calculation"},
  {.init = controllerCustomFirmware3Init, .test = controllerCustomFirmware3Test, .update = controllerCustomFirmware3, .name = "Custom Controller 3 - LQR"},
  // {.init = controllerCustomFirmware4Init, .test = controllerCustomFirmware4Test, .update = controllerCustomFirmware4, .name = "Custom Controller 4 - LQR with Ricatti Equation Solver"},
  // {.init = controllerCustomFirmware5Init, .test = controllerCustomFirmware5Test, .update = controllerCustomFirmware5, .name = "Custom Controller 5 - Sliding Mode Control other"},
  #ifdef CONFIG_CONTROLLER_OOT
  {.init = controllerOutOfTreeInit, .test = controllerOutOfTreeTest, .update = controllerOutOfTree, .name = "OutOfTree"},
  #endif
};


void controllerInit(ControllerType controller) {
  if (controller < 0 || controller >= ControllerType_COUNT) {
    return;
  }

  currentController = controller;

  if (ControllerTypeAutoSelect == currentController) {
    currentController = DEFAULT_CONTROLLER;
  }

  #if defined(CONFIG_CONTROLLER_PID)
    #define CONTROLLER ControllerTypePID
  #elif defined(CONFIG_CONTROLLER_INDI)
    #define CONTROLLER Cont#define CONTROLLER ControllerTypeCustom5   ollerTypeINDI
  #elif defined(CONFIG_CONTROLLER_MELLINGER)
    #define CONTROLLER ControllerTypeMellinger
  #elif defined(CONFIG_CONTROLLER_BRESCIANINI)
    #define CONTROLLER ControllerTypeBrescianini
  #elif defined(CONFIG_CONTROLLER_LEE)
    #define CONTROLLER ControllerTypeLee
  #elif defined(CONFIG_CONTROLLER_CUSTOM1)
    #define CONTROLLER ControllerTypeCustom1
  #elif defined(CONFIG_CONTROLLER_CUSTOM2)
    #define CONTROLLER ControllerTypeCustom2
  #elif defined(CONFIG_CONTROLLER_CUSTOM3)
    #define CONTROLLER ControllerTypeCustom3
  #elif defined(CONFIG_CONTROLLER_CUSTOM4)
    #define CONTROLLER ControllerTypeCustom4
  #elif defined(CONFIG_CONTROLLER_CUSTOM5)
    #define CONTROLLER ControllerTypeCustom5
  #elif defined(CONFIG_CONTROLLER_OOT)
    #define CONTROLLER ControllerTypeOot
  #else
    #define CONTROLLER ControllerTypeAutoSelect
  #endif

  ControllerType forcedController = CONTROLLER;
  if (forcedController != ControllerTypeAutoSelect) {
    DEBUG_PRINT("Controller type forced\n");
    currentController = forcedController;
  }

  initController();

  DEBUG_PRINT("Using %s (%d) controller\n", controllerGetName(), currentController);
}

ControllerType controllerGetType(void) {
  return currentController;
}

static void initController() {
  controllerFunctions[currentController].init();
}

bool controllerTest(void) {
  return controllerFunctions[currentController].test();
}

void controller(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t stabilizerStep) {
  controllerFunctions[currentController].update(control, setpoint, sensors, state, stabilizerStep);
}

const char* controllerGetName() {
  return controllerFunctions[currentController].name;
}
