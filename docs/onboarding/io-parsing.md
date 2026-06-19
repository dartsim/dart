# IO And Model Loading

Model loading and parser changes can affect downstream users and Gazebo
compatibility. For dependency cleanup, treat parser dependencies and installed
headers as compatibility surfaces.

Run focused IO tests when touching parser code, model-loading dependencies, or
package metadata used by IO components.
