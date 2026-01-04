import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import Range
import serial
import time

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # --- הגדרות חיבור ---
        # שים לב: בדוק בטרמינל איזה פורט הארדואינו קיבל (לרוב /dev/ttyUSB0 או /dev/ttyACM0)
        self.serial_port = '/dev/ttyUSB0' 
        self.baud_rate = 57600
        
        # אתחול חיבור Serial
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f'Connected to Arduino on {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            exit(1)

        # --- Publishers (שולחים מידע מהארדואינו ל-ROS) ---
        # מפרסמים את הטיקים של האנקודרים
        self.left_tick_pub = self.create_publisher(Int32, 'left_ticks', 10)
        self.right_tick_pub = self.create_publisher(Int32, 'right_ticks', 10)
        
        # מפרסמים את המרחק מהאולטרסוניק כהודעת Range תקנית
        self.range_pub = self.create_publisher(Range, 'ultrasonic_range', 10)

        # --- Subscribers (מקבלים פקודות מ-ROS לארדואינו) ---
        # מאזינים לפקודות תנועה (למשל מ-Teleop או Nav2)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        # --- Timer (לולאת קריאה) ---
        # נבדוק אם הגיע מידע מהארדואינו כל 50 מילי-שניות
        self.timer = self.create_timer(0.05, self.read_from_arduino)
        
        # פרמטרים פיזיים של הרובוט (לחישוב PWM בסיסי)
        self.wheel_separation = 0.20  # המרחק בין הגלגלים במטרים (נא למדוד!)
        self.wheel_radius = 0.03      # רדיוס הגלגל במטרים

    def cmd_vel_callback(self, msg):
        """
        פונקציה זו נקראת בכל פעם שמגיעה פקודת תנועה (Twist).
        היא ממירה מהירות ליניארית וזוויתית למהירות גלגלים (PWM).
        """
        linear = msg.linear.x
        angular = msg.angular.z
        
        # חישוב קינמטיקה דיפרנציאלית (Unicycle Model)
        # מהירות מבוקשת לכל גלגל במטר לשנייה
        left_speed_m_s = linear - (angular * self.wheel_separation / 2.0)
        right_speed_m_s = linear + (angular * self.wheel_separation / 2.0)
        
        # המרה ל-PWM (בין -255 ל 255)
        # הערה: זהו מיפוי פשוט (Open Loop). בהמשך עדיף לממש PID בארדואינו ולשלוח לו m/s ישירות.
        pwm_left = self.map_speed_to_pwm(left_speed_m_s)
        pwm_right = self.map_speed_to_pwm(right_speed_m_s)
        
        # שליחת הפקודה לארדואינו: "L,R\n"
        command = f"{pwm_left},{pwm_right}\n"
        self.ser.write(command.encode('utf-8'))
        # self.get_logger().info(f'Sent to Arduino: {command.strip()}')

    def map_speed_to_pwm(self, speed_m_s):
        """
        פונקציית עזר להמרת מטר/שנייה לערך PWM.
        דורש כיול! כרגע זה ניחוש לינארי.
        """
        if speed_m_s == 0:
            return 0
            
        # נניח שהרובוט מגיע למהירות מקסימלית של 0.5 מטר/שנייה ב-PWM 255
        max_speed = 0.5 
        pwm = int((speed_m_s / max_speed) * 255)
        
        # הגבלת הערכים לטווח המותר
        return max(min(pwm, 255), -255)

    def read_from_arduino(self):
        """
        פונקציה זו רצה בלולאה ומחפשת מידע שמגיע מה-Serial
        """
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                # הפורמט הצפוי מהארדואינו: LeftTicks,RightTicks,Distance
                parts = line.split(',')
                
                if len(parts) == 3:
                    l_ticks = int(parts[0])
                    r_ticks = int(parts[1])
                    dist_cm = float(parts[2])
                    
                    # פרסום הטיקים
                    self.left_tick_pub.publish(Int32(data=l_ticks))
                    self.right_tick_pub.publish(Int32(data=r_ticks))
                    
                    # פרסום נתוני החיישן (Range)
                    range_msg = Range()
                    range_msg.header.stamp = self.get_clock().now().to_msg()
                    range_msg.header.frame_id = "ultrasonic_link"
                    range_msg.radiation_type = Range.ULTRASOUND
                    range_msg.field_of_view = 0.26 # כ-15 מעלות ברדיאנים
                    range_msg.min_range = 0.02
                    range_msg.max_range = 4.0
                    range_msg.range = dist_cm / 100.0 # המרה למטרים
                    
                    self.range_pub.publish(range_msg)
                    
            except ValueError:
                pass # סינון הודעות זבל שעלולות להגיע בהתחלה
            except Exception as e:
                self.get_logger().warning(f'Serial read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()