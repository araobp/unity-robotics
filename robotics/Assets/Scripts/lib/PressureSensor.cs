using UnityEngine;

/// <summary>
/// Simulates a pressure sensor by calculating the force and pressure
/// exerted on this GameObject during a collision.
/// </summary>
public class PressureSensor : MonoBehaviour
{
    private float _lastForce;
    /// <summary>
    /// Gets the last calculated force of the collision in Newtons.
    /// </summary>
    public float LastForce => _lastForce;

    private float _lastPressure;
    /// <summary>
    /// Gets the last calculated pressure of the collision.
    /// </summary>
    public float LastPressure => _lastPressure;

    private float _lastMass;
    /// <summary>
    /// Gets the mass of the other rigidbody in the collision.
    /// </summary>
    public float LastMass => _lastMass;

    private float _lastFriction;
    /// <summary>
    /// Gets the average static friction of the two colliding objects.
    /// </summary>
    public float LastFriction => _lastFriction;

    private bool _isColliding;
    /// <summary>
    /// Gets a value indicating whether a collision is currently occurring.
    /// </summary>
    public bool IsColliding => _isColliding;

    private GameObject _contactObject;
    /// <summary>
    /// Gets the object that this sensor is colliding with.
    /// </summary>
    public GameObject ContactObject => _contactObject;

    /// <summary>
    /// Unity message for when a collision first occurs.
    /// </summary>
    /// <param name="collision">Collision event data.</param>
    private void OnCollisionEnter(Collision collision)
    {
        _isColliding = true;
        CalculateForceAndPressure(collision);
        if (collision.rigidbody != null)
            _lastMass = collision.rigidbody.mass;
        var col = GetComponent<Collider>();
        _lastFriction = (collision.collider.material.staticFriction + col.material.staticFriction) / 2.0f;
        _contactObject = collision.gameObject;
    }

    /// <summary>
    /// Unity message for every frame that a collision continues.
    /// This is used to continuously update pressure, for example, when one object is crushing another.
    /// </summary>
    /// <param name="collision">Collision event data.</param>
    private void OnCollisionStay(Collision collision)
    {
        CalculateForceAndPressure(collision);
        _contactObject = collision.gameObject;
    }

    /// <summary>
    /// Unity message for when a collision ends.
    /// </summary>
    /// <param name="collision">Collision event data.</param>
    private void OnCollisionExit(Collision collision)
    {
        // Reset values when the collision ends.
        _isColliding = false;
        _lastMass = 0f;
        _lastFriction = 0f;
        _lastForce = 0f;
        _lastPressure = 0f;
        _contactObject = null;
    }

    /// <summary>
    /// Calculates the force and pressure from collision data.
    /// </summary>
    /// <param name="collision">The collision data.</param>
    private void CalculateForceAndPressure(Collision collision)
    {
        // The total impulse of the collision, which is Force * Time.
        var impulse = collision.impulse.magnitude;

        // The number of contact points is used to approximate the contact area.
        var contactCount = collision.contactCount;

        // Ensure there are contact points to avoid division by zero.
        if (contactCount > 0)
        {
            // Force is approximated by dividing impulse by the fixed time step.
            // This gives us a value in Newtons.
            _lastForce = impulse / Time.fixedDeltaTime;

            // Pressure is Force / Area. We approximate Area with the number of contact points.
            // This is a simplification and may not be physically accurate.
            _lastPressure = _lastForce / contactCount;

            Debug.Log($"Force: {_lastForce:F2} N on {gameObject.name}");
            Debug.Log($"Pressure: {_lastPressure:F2} Pa on {gameObject.name}");
        }
    }
}