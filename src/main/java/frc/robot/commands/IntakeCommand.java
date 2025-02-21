// package frc.robot.commands;

// import static edu.wpi.first.units.Units.MetersPerSecond;

// import edu.wpi.first.units.measure.LinearVelocity;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.wrist.Wrist;

// public class IntakeCommand extends Command {

//   public LinearVelocity velocity;
//   public Intake intake;
//   public Wrist wrist;
//   public double wristAngle;
//   public Boolean sensor1Broken = null;
//   public Boolean sensor2Broken = null;
//   public Integer index;

//   public IntakeCommand(Intake intake) {
//     this.intake = intake;
//     this.wrist = null;
//     addRequirements(intake);
//     this.sensor1Broken = intake.getSensor1();
//     this.sensor2Broken = intake.getSensor2();
//     this.index = null;
//   }

//   public IntakeCommand(Intake intake, Wrist wrist) {
//     this.intake = intake;
//     this.wrist = wrist;
//     this.wristAngle = 2.2;
//     addRequirements(intake, wrist);
//     this.index = 0;
//   }

//   @Override
//   public void initialize() {
//     velocity = LinearVelocity.ofBaseUnits(0.0, MetersPerSecond);
//     ; // LinearVelocity.ofBaseUnits(intake.getConstants().intakeVelocity, MetersPerSecond);
//     // intake.setVelocity(velocity);
//     // if (wrist != null) {
//     //   wrist.goToAngle(wristAngle);
//     // }
//   }

//   @Override
//   public void execute() {
//     if (sensor1Broken != null && sensor2Broken != null) {
//       if (intake.getSensor1() == true && intake.getSensor2() == false) {
//         this.cancel();
//       } else if (intake.getSensor1() == false) {
//         setVelocity(LinearVelocity.ofBaseUnits(12, MetersPerSecond));
//         sensor2Broken = true;
//       }
//     }
//     if (index != null && index == 0) {
//       velocity = LinearVelocity.ofBaseUnits(0.0, MetersPerSecond);
//       intake.setVelocity(velocity);
//       if (wrist != null) {
//         wrist.goToAngle(0);
//       }
//     } else if (index != null && index == 1) {
//       velocity = LinearVelocity.ofBaseUnits(Constants.AlgaeIntake.intakeVelocity, MetersPerSecond);
//       intake.setVelocity(velocity);
//       if (wrist != null) {
//         wrist.goToAngle(wristAngle);
//       }
//     } else if (index != null && index == 2) {
//       velocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
//       if (wrist.getWristPosition() <= 0.8 + 0.1) {
//         intake.runVoltage(1);
//       }
//       if (wrist != null) {
//         wrist.goToAngle(0.8);
//       }
//     } else {
//       intake.setVelocity(velocity);
//     }
//   }

//   @Override
//   public void end(boolean interrupted) {
//     intake.setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
//     if (wrist != null) {
//       intake.runVoltage(1);
//       if (Robot.robotContainer.wristIsDown) {
//         wrist.goToAngle(0);
//         intake.setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
//       }
//     }
//   }

//   public void index() {
//     if (index != null) {
//       if (index == 2) {
//         index = 0;
//       } else {
//         index++;
//       }
//     }
//   }

//   public void setVelocity(LinearVelocity velocity) {
//     this.velocity = velocity;
//   }
// }
// // test commit
