//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#include "SimpleFEMDefs.h"
#include "SimpleFEM.h"
#include "MeshViewer.h"
#include <iomanip>

// size of grid
static const int gridSize = 32;
// use a graded mesh, or a regular mesh
static const bool gradedMesh = true;
// laplace or poisson problem?
static const bool laplaceProblem = false;
// display debug information?
static const bool debugOut = false;


double eval_u(double x, double y)
{
	if (laplaceProblem)
		return exp(x) * sin(y);
	else
		return 	3 * x * x + 2 * y * y * y * x;
}

double eval_f(double x, double y)
{
	if (laplaceProblem)
		return 0;
	else
		return -6 - 12 * y * x;
}

FEMMesh SimpleFEM::CreateUniformGridMesh(int nodesX, int nodesY)
{
	assert(nodesX >= 2);
	assert(nodesY >= 2);

	FEMMesh mesh;

	// Setup positions of nodes
	int nodecnt = 0;
	for (int y = 0; y < nodesY; y++)
	{
		for (int x = 0; x < nodesX; x++)
		{
			Vector2 pos = Vector2((double)x / (double)(nodesX - 1), (double)y / (double)(nodesY - 1));

			// Shift mesh positions for graded mesh
			if (gradedMesh)
			{
				pos[0] *= pos[0];
				pos[1] *= pos[1];
			}
			mesh.AddNode(pos);
			nodecnt++;
		}
	}
	std::cout << "Added " << nodecnt << " nodes to mesh.\n";

	// Create elements
	int cnt = 0;
	for (int y = 0; y < nodesY - 1; y++)
	{
		for (int x = 0; x < nodesX - 1; x++)
		{
			// bottom-left:
			int node00 = y*nodesX + x;

			// bottom-right:
			int node10 = node00 + 1;

			// top-left:
			int node01 = node00 + nodesX;

			// top-right:
			int node11 = node00 + nodesX + 1;

			// add two element for this quad
			mesh.AddElement(FEMElementTri(node00, node10, node11));
			mesh.AddElement(FEMElementTri(node00, node11, node01));
			cnt += 2;
		}
	}
	std::cout << "Added " << cnt << " elements to mesh.\n";

	return mesh;
}

void SimpleFEM::ComputeBoundaryConditions(const FEMMesh &mesh, std::vector<BoundaryCondition> &boundaryConditions)
{
	boundaryConditions.clear();

	for (int nodeID = 0; nodeID < mesh.GetNumNodes(); nodeID++)
	{
		const Vector2 &pos = mesh.GetNodePosition(nodeID);

		if (isOnBoundary(pos)) {
			double x = pos[0];
			double y = pos[1];

			// compute reference solution on boundary
			double val = eval_u(x, y);

			// this fixes the solution for node "nodeID" to "val" when
			// solving the system later on
			boundaryConditions.push_back(BoundaryCondition(nodeID, val));
		}
	}
}

// perform a simple boundary check
// is either of the components 0 or 1?
bool SimpleFEM::isOnBoundary(const Vector2 &pos)
{
	return pos[0] <= 0. || pos[0] >= 1. || pos[1] <= 0. || pos[1] >= 1.;
}

// TASK 4
void SimpleFEM::ComputeRHS(const FEMMesh &mesh, std::vector<double> &rhs)
{
	for (int ie = 0; ie < mesh.GetNumElements(); ie++)
	{
		const FEMElementTri& elem = mesh.GetElement(ie);

		//Task4 starts here
		double area;
		elem.computeElementArea(&mesh, area);
		Vec2 barycenter = (mesh.GetNodePosition(elem.GetGlobalNodeForElementNode(0)) + mesh.GetNodePosition(elem.GetGlobalNodeForElementNode(1)) + mesh.GetNodePosition(elem.GetGlobalNodeForElementNode(2))) / 3.;
		for (int j = 0; j < 3; ++j)
		{
			rhs.at(elem.GetGlobalNodeForElementNode(j)) += area*eval_f(barycenter.x(), barycenter.y())*1./3; //does the same as below due to regularity of triangles
			//rhs.at(elem.GetGlobalNodeForElementNode(j)) += area*eval_f(barycenter.x(), barycenter.y())*elem.evalSingleBasisGlobalLES(j, &mesh, barycenter.x(), barycenter.y()); 
		}
		//Task4 ends here
	}
}

// TASK 5
void SimpleFEM::computeError(FEMMesh &mesh, const std::vector<double> &sol_num, std::vector<double> &verror, double &err_nrm)
{
	//Task 5 starts here
	for (unsigned i = 0; i < sol_num.size(); ++i)
	{
		verror.at(i) = std::abs(sol_num.at(i) - eval_u(mesh.GetNodePosition(i).x(), mesh.GetNodePosition(i).y()));
	}
	err_nrm = 0;
	for (unsigned i = 0; i < verror.size(); ++i)
	{
		double temp = 0;
		for (unsigned j = 0; j < verror.size(); ++j)
		{
			if (i >= j)
			{
				temp += verror.at(j)*mesh.getMat().GetAt(i, j);
			}
			else
			{
				temp += verror.at(j)*mesh.getMat().GetAt(j,i);
			}
		}
		err_nrm += temp*verror.at(i);
	}

	err_nrm = std::sqrt(err_nrm);
	/*
	std::vector<double> res1(verror.size());
	mesh.getMat().MultVector(verror, res1);
	for (int i = 0; i < verror.size(); ++i)
	{
		err_nrm += res1.at(i)*verror.at(i);
	}
	err_nrm = std::sqrt(err_nrm);*/
	//Task 5 ends here
}

int main(int argc, char *argv[])
{
	// Create a uniform mesh:
	FEMMesh mesh = SimpleFEM::CreateUniformGridMesh(gridSize, gridSize);
	int nNodes = mesh.GetNumNodes();


	// Build its stiffness matrix:
	// loop over all elements, and compute their contributions
	// for the equations of their respective nodes

	mesh.getMat().ClearResize(mesh.GetNumNodes());
	for (int i = 0; i < mesh.GetNumElements(); i++)
	{
		if (debugOut)
			std::cout << "Assembling " << i << "\n";
		mesh.GetElement(i).Assemble(&mesh);
	}

	// Compute boundary conditions and right-hand side:
	std::vector<BoundaryCondition> boundaryConditions;

	SimpleFEM::ComputeBoundaryConditions(mesh, boundaryConditions);

	// Apply right-hand side:
	std::vector<double> rhs(nNodes);
	SimpleFEM::ComputeRHS(mesh, rhs);
	mesh.SetRHS(rhs);

	// Solve the problem, this calls a preconditioned CG solver
	// for the sparse matrix with right hand side rhs
	// all nodes stored in "boundaryConditions" are fixed to certain values
	std::vector<double> solution;
	bool isSolved = mesh.Solve(solution, boundaryConditions);
	assert(isSolved);

	// Debug output: Print matrix for non-boundary nodes
	if (debugOut)
	{
		for (int i = 0; i < mesh.GetNumNodes(); i++)
		{
			const Vector2 & pi = mesh.GetNodePosition(i);
			if (SimpleFEM::isOnBoundary(pi))
				continue;
			for (int j = 0; j < mesh.GetNumNodes(); j++)
			{
				const Vector2 & pj = mesh.GetNodePosition(j);
				if (SimpleFEM::isOnBoundary(pj))
					continue;
				if (j > i)
					std::cout << mesh.getMat()(j, i) << "\t";
				else
					std::cout << mesh.getMat()(i, j) << "\t";
			}
			std::cout << std::endl;

			//std::cout << " rhs=" << rhs[i] << "\n";
		}
	}

	double err_nrm = 0;
	std::vector<double> verr(nNodes);
	SimpleFEM::computeError(mesh, solution, verr, err_nrm);
	std::cout << std::setprecision(6) << "Error norm is " << err_nrm << std::endl;
	// Visualize the solution:
	// draw the triangles with colors according to solution
	// blue means zero, red means maxValue.
	// the default problem goes from 0-5 , for other problems, 
	// adjust the maxValue parameter below (values <0, or >maxValue
	// are clamped for the display)
	MeshViewer viewer(argc, argv);

	viewer.InitializeVisualization();
	viewer.CreateSolutionVisualization(mesh, solution);
	viewer.CreateErrorVisualization(mesh, verr);
	viewer.RunVisualization();

	return 0;
}
