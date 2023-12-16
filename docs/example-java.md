# LaserCAN - FRC Example (Java)

## Install the Vendor Library:
In VSCode, hit CTRL+SHIFT+P and select "WPILib: Manage Vendor Libraries".  Select "Install new libraries (offline)" and paste the following URL: https://storage.googleapis.com/grapple-frc-maven/libgrapplefrc2023.json

## Use it!
```java
package frc.robot;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  private LaserCan lc;

  @Override
  public void robotInit() {
    lc = new LaserCan(0);
    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
    lc.setRangingMode(LaserCan.RangingMode.SHORT);
    lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
    lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
  }

  @Override
  public void robotPeriodic() {
    LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      System.out.println("The target is " + measurement.distance_mm + "mm away!");
    } else {
      System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
```