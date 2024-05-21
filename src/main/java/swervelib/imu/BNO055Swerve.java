package swervelib.imu;

import swervelib.imu.BNO055;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class BNO055Swerve extends SwerveIMU
{

  private BNO055     imu;
  private I2C.Port   port;
  /**
   * Offset for the bno055.
   */
  private Rotation3d offset      = new Rotation3d();
  /**
   * Inversion for the gyro
   */
  private boolean    invertedIMU = false;

  /**
   * Generate the SwerveIMU for bno055
   *
   * @param port i2c port for the bno055
   */
  public BNO055Swerve(I2C.Port port)
  {
    this.port = port;
    imu = BNO055.getInstance(
      BNO055.opmode_t.OPERATION_MODE_IMUPLUS, 
      BNO055.vector_type_t.VECTOR_EULER, 
      port,
      BNO055.BNO055_ADDRESS_A
    ); 
    
    // TODO: fix this bit
    SmartDashboard.putData(imu);
  }

  /**
   * Reset IMU to factory default.
   */
  @Override
  public void factoryDefault()
  {
    // no defaults to reset
  }

  /**
   * Clear sticky faults on IMU.
   */
  @Override
  public void clearStickyFaults()
  {
    // no sticky faults to clear
  }

  /**
   * Set the gyro offset.
   *
   * @param offset gyro offset as a {@link Rotation3d}.
   */
  public void setOffset(Rotation3d offset)
  {
    this.offset = offset;
  }

  /**
   * Set the gyro to invert its default direction
   *
   * @param invertIMU invert gyro direction
   */
  public void setInverted(boolean invertIMU)
  {
    invertedIMU = invertIMU;
  }

  /**
   * Fetch the {@link Rotation3d} from the IMU without any zeroing. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  @Override
  public Rotation3d getRawRotation3d()
  {
    imu = BNO055.getInstance(
      BNO055.opmode_t.OPERATION_MODE_IMUPLUS, 
      BNO055.vector_type_t.VECTOR_EULER, 
      port,
      BNO055.BNO055_ADDRESS_A
    );
    double[] xyz = imu.getVector();
    Rotation3d reading = new Rotation3d(xyz[1], xyz[2], xyz[0]);
    return invertedIMU ? reading.unaryMinus() : reading;
  }

  /**
   * Fetch the {@link Rotation3d} from the IMU. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  @Override
  public Rotation3d getRotation3d()
  {
    return getRawRotation3d().minus(offset);
  }

  /**
   * Fetch the acceleration [x, y, z] from the IMU in meters per second squared. If acceleration isn't supported returns
   * empty.
   *
   * @return {@link Translation3d} of the acceleration as an {@link Optional}.
   */
  @Override
  public Optional<Translation3d> getAccel()
  {
    imu = BNO055.getInstance(
      BNO055.opmode_t.OPERATION_MODE_IMUPLUS, 
      BNO055.vector_type_t.VECTOR_EULER, 
      port,
      BNO055.BNO055_ADDRESS_A
    );
    double[] xyz = imu.getVector();
    return Optional.of(new Translation3d(xyz[0], xyz[1], xyz[2]));
  }

  /**
   * Get the instantiated IMU object.
   *
   * @return IMU object.
   */
  @Override
  public Object getIMU()
  {
    return imu;
  }
}
