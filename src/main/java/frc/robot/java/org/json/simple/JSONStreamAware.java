package frc.robot.java.org.json.simple;

import java.io.IOException;
import java.io.Writer;

public interface JSONStreamAware {
  void writeJSONString(Writer out) throws IOException;
}
