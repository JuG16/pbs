//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#include "SimpleFEMDefs.h"
#include "FEMElementTri.h"
#include "FEMMesh.h"

// TASK 3
void FEMElementTri::Assemble(FEMMesh *pMesh) const
{
	
}

// TASK 2
void FEMElementTri::computeSingleBasisDerivGlobalGeom(int nodeId, Vector2 &basisDerivGlobal, const FEMMesh *pMesh) const
{
	
}

// TASK 1
void FEMElementTri::computeSingleBasisDerivGlobalLES(int nodeId, Vector2 &basisDerivGlobal, const FEMMesh *pMesh) const
{
	Matrix3x3 paramMatrix;

	paramMatrix(0, 0) = (*pMesh).GetNodePosition(this->GetGlobalNodeForElementNode(0)).x();
	paramMatrix(0, 1) = (*pMesh).GetNodePosition(this->GetGlobalNodeForElementNode(0)).y();
	paramMatrix(0, 2) = 1;
	paramMatrix(1, 0) = (*pMesh).GetNodePosition(this->GetGlobalNodeForElementNode(1)).x();
	paramMatrix(1, 1) = (*pMesh).GetNodePosition(this->GetGlobalNodeForElementNode(1)).y();
	paramMatrix(1, 2) = 1;
	paramMatrix(2, 0) = (*pMesh).GetNodePosition(this->GetGlobalNodeForElementNode(2)).x();
	paramMatrix(2, 1) = (*pMesh).GetNodePosition(this->GetGlobalNodeForElementNode(2)).y();
	paramMatrix(2, 2) = 1;

	Vector3 rhs;
	rhs.setX(this->GetGlobalNodeForElementNode(0) == nodeId ? 1 : 0);
	rhs.setY(this->GetGlobalNodeForElementNode(1) == nodeId ? 1 : 0);
	rhs.setZ(this->GetGlobalNodeForElementNode(2) == nodeId ? 1 : 0);


	Vector3 params = paramMatrix.inverse()*rhs;

	basisDerivGlobal.x() = params.x();
	basisDerivGlobal.y() = params.y();
}
