#pragma once

// Wrapper for Shewchuk's exact geometric predicates (predicates.c).
// Used for robust point/triangle and point/tetrahedron predicates.

extern "C"
{
	void exactinit();
	double orient2d(const double* Pa, const double* Pb, const double* Pc);
	double orient3d(const double* Pa, const double* Pb, const double* Pc, const double* Pd);
	double incircle(const double* Pa, const double* Pb, const double* Pc, const double* Pd);
	double insphere(const double* Pa, const double* Pb, const double* Pc, const double* Pd, const double* Pe);
}
