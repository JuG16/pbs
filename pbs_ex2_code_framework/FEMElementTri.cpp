//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#include "SimpleFEMDefs.h"
#include "FEMElementTri.h"
#include "FEMMesh.h"

//Compute Area
void FEMElementTri::computeElementArea(const FEMMesh *pMesh, double &area) const
{

	area=std::abs((pMesh->GetNodePosition(this->GetGlobalNodeForElementNode(1)).x()- pMesh->GetNodePosition(this->GetGlobalNodeForElementNode(0)).x())*(pMesh->GetNodePosition(this->GetGlobalNodeForElementNode(2)).y() - pMesh->GetNodePosition(this->GetGlobalNodeForElementNode(0)).y())- (pMesh->GetNodePosition(this->GetGlobalNodeForElementNode(1)).y() - pMesh->GetNodePosition(this->GetGlobalNodeForElementNode(0)).y())*(pMesh->GetNodePosition(this->GetGlobalNodeForElementNode(2)).x() - pMesh->GetNodePosition(this->GetGlobalNodeForElementNode(0)).x()))/2.;
}

// TASK 3
void FEMElementTri::Assemble(FEMMesh *pMesh) const
{
	double area;
	this->computeElementArea(pMesh, area);
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (this->GetGlobalNodeForElementNode(i) >= this->GetGlobalNodeForElementNode(j))
			{
				Vec2 basisDerivi;
				Vec2 basisDerivj;
				this->computeSingleBasisDerivGlobalLES(i, basisDerivi, pMesh);
				this->computeSingleBasisDerivGlobalLES(j, basisDerivj, pMesh);
				pMesh->AddToStiffnessMatrix(this->GetGlobalNodeForElementNode(i), this->GetGlobalNodeForElementNode(j), area*(basisDerivi.x()*basisDerivj.x() + basisDerivi.y()*basisDerivj.y()));
			}
		}
	}
}

// TASK 2
void FEMElementTri::computeSingleBasisDerivGlobalGeom(int nodeId, Vector2 &basisDerivGlobal, const FEMMesh *pMesh) const
{
	
}

// TASK 1
void FEMElementTri::computeSingleBasisDerivGlobalLES(int nodeId, Vector2 &basisDerivGlobal, const FEMMesh *pMesh) const
{
	//does nodeId refere to local or global numbering (assuming local for now)
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
	rhs.setX(0 == nodeId ? 1 : 0);
	rhs.setY(1 == nodeId ? 1 : 0);
	rhs.setZ(2 == nodeId ? 1 : 0);


	Vector3 params = paramMatrix.inverse()*rhs;

	basisDerivGlobal.x() = params.x();
	basisDerivGlobal.y() = params.y();
}
