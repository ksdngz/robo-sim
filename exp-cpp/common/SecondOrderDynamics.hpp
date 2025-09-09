#pragma once

// Second-order-delayed system (Position type supported)
class SecondOrderDynamics {
public:
	// Parameters: inertia m, damping ratio zeta, natural angular frequency omega
	double m;
	double zeta;
	double omega; // rad/s
	Position y;    // Output (position)
	Position yd;   // First derivative (velocity)

	SecondOrderDynamics(double m_, double zeta_, double omega_, const Position& initp)
		: m(m_), zeta(zeta_), omega(omega_), y(initp), yd{0,0,0} {}

	// ref: reference input, dt: control timestep
	// Continuous system: y'' + 2*zeta*omega*y' + omega^2 * y = omega^2 * ref
	// Numerical integration: Forward Euler (extendable to Heun / Runge-Kutta if needed)
	Position update(const Position& ref, double dt) {
		// ydd = ω^2 (ref - y) - 2 ζ ω yd
		Position ydd = (ref - y) * (omega * omega) - yd * (2.0 * zeta * omega);
		yd += ydd * dt;
		y  += yd * dt;
		return y;
	}
};
