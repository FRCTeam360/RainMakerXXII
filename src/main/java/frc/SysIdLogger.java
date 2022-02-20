package frc;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SysIdLogger {
    String m_mechanism;
    String m_testType;
    boolean m_rotate;
    double m_voltageCommand;
    double m_startTime;
    double m_timestamp;
    ArrayList<Double> m_data = new ArrayList<Double>(kDataVectorSize);
    double m_motorVoltage;

    static final int kThreadPriority = 15;
    static final int kHALThreadPriority = 40;
    // 20 seconds of test data * 200 samples/second * 9 doubles/sample (320kB of
    // reserved data) provides a large initial vector size to avoid reallocating
    // during a test
    static final int kDataVectorSize = 36000;

    private Boolean isWrongMechanism () { return false; }

    public void initLogging() {
        m_mechanism = SmartDashboard.getString("SysIdTest", "");
      
        if (isWrongMechanism()) {
          SmartDashboard.putBoolean("SysIdWrongMech", true);
        }
      
        m_testType = SmartDashboard.getString("SysIdTestType", "");
        m_rotate = SmartDashboard.getBoolean("SysIdRotate", false);
        m_voltageCommand = SmartDashboard.getNumber("SysIdVoltageCommand", 0.0);
        m_startTime = Timer.getFPGATimestamp();
        m_data.clear();
      }
      
      public void sendData() {
        System.out.format("Collected: {} data points.\n", m_data.size());
      
        SmartDashboard.putBoolean("SysIdOverflow", false); // can't overflow a java ArrayList
        String ss = m_data.toString();
      
        SmartDashboard.putString("SysIdTelemetry", ss.substring(1, ss.length()-1));
      
        reset();
      }
      
      public void updateThreadPriority() {
        if (!RobotBase.isSimulation()) {
          if (!Notifier.setHALThreadPriority(true, kHALThreadPriority) ||
              !Threads.setCurrentThreadPriority(true, kThreadPriority)) {
            System.out.println("Setting the RT Priority failed\n");
          }
        }
      }
      
      public SysIdLogger() {
        System.out.println("Initializing logger\n");
        m_data.ensureCapacity(kDataVectorSize);
        LiveWindow.disableAllTelemetry();
      }
      
      public void updateData() {
        m_timestamp = Timer.getFPGATimestamp();
      
        // Don't let robot move if it's characterizing the wrong mechanism
        if (!isWrongMechanism()) {
          if (m_testType.equals("Quasistatic")) {
            m_motorVoltage = m_voltageCommand * (m_timestamp - m_startTime);
          } else if (m_testType.equals("Dynamic")) {
            m_motorVoltage = m_voltageCommand;
          } else {
            m_motorVoltage = 0.0;
          }
        } else {
          m_motorVoltage = 0.0;
        }
      }
      
      void reset() {
        m_motorVoltage = 0.0;
        m_timestamp = 0.0;
        m_startTime = 0.0;
        m_data.clear();
      }
      
    
}