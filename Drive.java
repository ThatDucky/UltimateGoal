import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.lang.Math;

@TeleOp(name = "Drive", group = "TeleOp")

public class Drive extends OpMode {
        Hardware robot = new Hardware();
        @Override
        public void init() {

        }

        @Override
        public void init_loop() {

        }

        @Override
        public void start() {

        }

        @Override
        public void loop() {
                robot.init(hardwareMap);

        }
}
