package frc.robot.subsystems.localization;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Objects;

@SuppressWarnings("MemberName")
public class NodePosition {
  @JsonProperty(value = "position")
  public int Position;

  @JsonProperty(value = "pose")
  public Pose3d pose;

  @SuppressWarnings("ParameterName")
  @JsonCreator
  public NodePosition(
      @JsonProperty(required = true, value = "Position") int Position,
      @JsonProperty(required = true, value = "pose") Pose3d pose) {
    this.Position = Position;
    this.pose = pose;
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof NodePosition) {
      var other = (NodePosition) obj;
      return Position == other.Position && pose.equals(other.pose);
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(Position, pose);
  }

  @Override
  public String toString() {
    return "NodePosition(Position: " + Position + ", pose: " + pose + ")";
  }
}
