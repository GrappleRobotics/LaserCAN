# LaserCAN - FRC Example (C++)

## Install the Vendor Library:
In VSCode, hit CTRL+SHIFT+P and select "WPILib: Manage Vendor Libraries".  Select "Install new libraries (offline)" and paste the following URL: https://storage.googleapis.com/grapple-frc-maven/libgrapplefrc2023.json

## Use it!
`Robot.h`

```c++
#pragma once

#include <frc/TimedRobot.h>
#include <libgrapplefrc/LaserCan.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;
 private:
  libgrapplefrc::LaserCan *lc;
};
```

`Robot.cpp`

```c++
#include "Robot.h"
#include <iostream>

void Robot::RobotInit() {
  lc = new libgrapplefrc::LaserCan(0);
    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
  lc->set_ranging_mode(libgrapplefrc::LaserCanRangingMode::Long);
  lc->set_timing_budget(libgrapplefrc::LaserCanTimingBudget::TimingBudget100ms);
  lc->set_roi(libgrapplefrc::LaserCanROI{ 8, 8, 16, 16 });
}

void Robot::RobotPeriodic() {
  libgrapplefrc::LaserCanStatus status = lc->status();
  if (status.status == 0) {
    std::cout << "The target is " << status.distance_mm << "mm away!" << std::endl;
  } else {
    std::cout << "Oh no! The target is out of range, or we can't get a reliable measurement!" << std::endl;
  }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

```