package frc.robot.subsystems.localization;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Objects;

@SuppressWarnings("MemberName")
public class NodePosition {
  @JsonProperty(value = "position")
  // The positions in the json file is this int position
  public int position;

  @JsonProperty(value = "pose")
  public Pose3d pose;

  @SuppressWarnings("ParameterName")
  @JsonCreator
  public NodePosition(
      @JsonProperty(required = true, value = "position") int position,
      @JsonProperty(required = true, value = "pose") Pose3d pose) {
    this.position = position;
    this.pose = pose;
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof NodePosition) {
      var other = (NodePosition) obj;
      return position == other.position && pose.equals(other.pose);
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(position, pose);
  }

  @Override
  public String toString() {
    return "NodePosition(position: " + position + ", pose: " + pose + ")";
  }
}
