using UnityEngine;

/// <summary>
/// A MonoBehaviour that simulates a pressure sensor. It calculates the force and pressure
/// exerted on the GameObject during a collision.
/// </summary>
public class PressureSensor : MonoBehaviour
{
    /// <summary>
    /// A threshold for pressure. If the calculated pressure exceeds this value,
    /// it could trigger an event. Currently not used in this script.
    /// </summary>
    public float pressureThreshold = 10f;

    /// <summary>
    /// Backing field for the last calculated force in Newtons.
    /// </summary>
    private float _lastForce = 0f;

    /// <summary>
    /// Public property to get the last calculated force.
    /// </summary>
    public float LastForce
    {
        get { return _lastForce; }
    }

    /// <summary>
    /// Backing field for the last calculated pressure.
    /// </summary>
    private float _lastPressure = 0f;

    /// <summary>
    /// Public property to get the last calculated pressure.
    /// </summary>
    public float LastPressure
    {
        get { return _lastPressure; }
    }

    private float _lastMass = 0f;
    public float LastMass
    {
        get { return _lastMass; }
    }

    private float _lastFriction = 0f;
    public float LastFriction
    {
        get { return _lastFriction; }
    }

    public bool _onCollisionEntered = false;
    public bool OnCollisionEntered
    {
        get { return _onCollisionEntered; }
    }

    /// <summary>
    /// Called by Unity when a collision first occurs.
    /// </summary>
    /// <param name="collision">Collision event data.</param>
    void OnCollisionEnter(Collision collision)
    {
        CalculatePressure(collision);
        _lastMass = collision.rigidbody.mass;
        Collider col = GetComponent<Collider>();
        _lastFriction = (collision.collider.material.staticFriction + col.material.staticFriction)/2.0f;
        _onCollisionEntered = true;
    }

    /// <summary>
    /// Called by Unity on every frame that a collision continues.
    /// </summary>
    /// <param name="collision">Collision event data.</param>
    void OnCollisionStay(Collision collision)
    {
        // Use Stay if you want to detect pressure while 
        // one object is crushing another.
        CalculatePressure(collision);
    }

    void OnCollisionExit(Collision collision)
    {
        _onCollisionEntered = false;
        _lastMass = 0f;
        _lastFriction = 0f;
        _lastForce = 0f;
        _lastPressure = 0f;
    }

    /// <summary>
    /// Calculates the force and pressure based on collision data.
    /// </summary>
    /// <param name="collision">The collision data.</param>
    void CalculatePressure(Collision collision)
    {
        // 1. Get the total impulse (Force * Time) of the impact
        float impulse = collision.impulse.magnitude;

        // 2. Get the number of contact points (Approximating Area)
        int contactCount = collision.contactCount;

        // Ensure there are contact points to avoid division by zero.
        if (contactCount > 0)
        {
            // 3. Pressure = Force / Area
            // We divide by Time.fixedDeltaTime to turn Impulse into an approximation of force in Newtons.
            _lastForce = impulse / Time.fixedDeltaTime;
            // We approximate the area of contact by the number of contact points.
            _lastPressure = _lastForce / contactCount;

            Debug.Log($"Force detected: {_lastForce} on {gameObject.name}");
            Debug.Log($"Pressure detected: {_lastPressure} on {gameObject.name}");
        }
    }
}