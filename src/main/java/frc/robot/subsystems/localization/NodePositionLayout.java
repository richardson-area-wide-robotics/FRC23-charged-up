package frc.robot.subsystems.localization;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

/**
 * Class for representing a layout of nodePositions on a field and reading them from a JSON format.
 *
 * <p>The JSON format contains two top-level objects, "positions" and "field". The "positions" object is a
 * list of all nodePositions contained within a layout. Each NodePosition serializes to a JSON object
 * containing an ID and a Pose3d. The "field" object is a descriptor of the size of the field in
 * meters with "width" and "length" values. This is to account for arbitrary field sizes when
 * transforming the poses.
 *
 * <p>Pose3ds in the JSON are measured using the normal FRC coordinate system, NWU with the origin
 * at the bottom-right corner of the blue alliance wall. {@link #setOrigin(OriginPosition)} can be
 * used to change the poses returned from {@link NodePositionLayout#getPositionPose(int)} to be from the
 * perspective of a specific alliance.
 *
 * <p>Position poses represent the center of the position, with a zero rotation representing a position that is
 * upright and facing away from the (blue) alliance wall (that is, towards the opposing alliance).
 */
@JsonIgnoreProperties(ignoreUnknown = true)
@JsonAutoDetect(getterVisibility = JsonAutoDetect.Visibility.NONE)
public class NodePositionLayout {

  private final Map<Integer, NodePosition> m_nodePositions = new HashMap<>();

  private Pose3d m_origin;

  /**
   * Construct a new NodePositionLayout with values imported from a JSON file.
   *
   * @param path Path of the JSON file to import from.
   * @throws IOException If reading from the file fails.
   */
  public NodePositionLayout(String path) throws IOException {
    this(Path.of(path));
  }

  /**
   * Construct a new NodePositionLayout with values imported from a JSON file.
   *
   * @param path Path of the JSON file to import from.
   * @throws IOException If reading from the file fails.
   */
  public NodePositionLayout(Path path) throws IOException {
    try{
    NodePositionLayout layout =
        new ObjectMapper().readValue(path.toFile(), NodePositionLayout.class);
    m_nodePositions.putAll(layout.m_nodePositions);
  }
  catch(IOException ex)
  {
    System.err.println("io exception " + ex);
    throw ex;
  }
  }

  @JsonCreator
  private NodePositionLayout(
      @JsonProperty(required = true, value = "positions") List<NodePosition> nodePositions) {
    // To ensure the underlying semantics don't change with what kind of list is passed in
    for (NodePosition position : nodePositions) {
      m_nodePositions.put(position.position, position);
    }
  }

  /**
   * Returns a List of the {@link NodePosition nodePositions} used in this layout.
   *
   * @return The {@link NodePosition nodePositions} used in this layout.
   */
  @JsonProperty("positions")
  public List<NodePosition> getPositions() {
    return new ArrayList<>(m_nodePositions.values());
  }


  /**
   * Gets an NodePosition pose by its ID.
   *
   * @param ID The ID of the position.
   * @return The pose corresponding to the ID passed in or an empty optional if a position with that ID
   *     was not found.
   */
  @SuppressWarnings("ParameterName")
  public Optional<Pose3d> getPositionPose(int ID) {
    NodePosition position = m_nodePositions.get(ID);
    if (position == null) {
      return Optional.empty();
    }
    return Optional.of(position.pose.relativeTo(m_origin));
  }

  /**
   * Serializes a NodePositionLayout to a JSON file.
   *
   * @param path The path to write to.
   * @throws IOException If writing to the file fails.
   */
  public void serialize(String path) throws IOException {
    serialize(Path.of(path));
  }

  /**
   * Serializes a NodePositionLayout to a JSON file.
   *
   * @param path The path to write to.
   * @throws IOException If writing to the file fails.
   */
  public void serialize(Path path) throws IOException {
    new ObjectMapper().writeValue(path.toFile(), this);
  }

  /**
   * Deserializes a field layout from a resource within a jar file.
   *
   * @param resourcePath The absolute path of the resource
   * @return The deserialized layout
   * @throws IOException If the resource could not be loaded
   */
  public static NodePositionLayout loadFromResource(String resourcePath) throws IOException {
    try (InputStream stream = NodePositionLayout.class.getResourceAsStream(resourcePath);
        InputStreamReader reader = new InputStreamReader(stream)) {
      return new ObjectMapper().readerFor(NodePositionLayout.class).readValue(reader);
    }
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof NodePositionLayout) {
      var other = (NodePositionLayout) obj;
      return m_nodePositions.equals(other.m_nodePositions) && m_origin.equals(other.m_origin);
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(m_nodePositions, m_origin);
  }
}
