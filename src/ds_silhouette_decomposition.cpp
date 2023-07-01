#include <iostream>
#include <iomanip>
#include <math.h>

#include "ds_silhouette_decomposition.h"

//----------------------------------------
//Constructors
gpcc::SilhouetteDecomposition::SilhouetteDecomposition() :
	nBits(0)
{
}

gpcc::SilhouetteDecomposition::SilhouetteDecomposition(uint32_t nbits):
	nBits(nbits)
{
}

//----------------------------------------
//Public Functions
void gpcc::SilhouetteDecomposition::createDyadicDecomposition()
{
	uint32_t iStart = 0;
	uint32_t iEnd   = (1 << this->nBits) - 1;

	//Create the father Silhouette
	int32_t myID = xGetID(iStart, iEnd);
	//std::cout << myID << " : (" << iStart << " , " << iEnd << " )" << std::endl;

	Silhouette s;
	s.iStart = iStart;
	s.iEnd = iEnd;
	s.myID = myID;
	s.fatherID = NONE;
	s.leftBrotherID = NONE;
	s.isRightBrother = false;

	this->silhouette_list.push_back(s);

	this->xCreateDyadicDecomposition(iStart, iEnd);

}


void gpcc::SilhouetteDecomposition::printList()
{
	std::cout << "Silhouette List" << std::endl;
	std::cout << " iD |   range   | fatherID | leftBrotherID | isRightBrother " << std::endl;
	for (auto& s : silhouette_list)
	{
		std::cout << std::setw(3) << s.myID << " | ";
		std::cout << "(" << std::setw(3) << s.iStart << "," << std::setw(3) << s.iEnd << ") | ";
		std::cout << std::setw(5) << s.fatherID << "    | ";
		std::cout << std::setw(7) << s.leftBrotherID << "       | ";
		std::cout << std::setw(7) << (s.isRightBrother ? 1 : 0) << std::endl;
	}
}

//----------------------------------------
//Private Functions
void gpcc::SilhouetteDecomposition::xCreateDyadicDecomposition(uint32_t iStart, uint32_t iEnd)
{
	Silhouette sLeft, sRight;
	uint32_t lStart, lEnd, rStart, rEnd;
	// uint32_t lStart, lEnd, lID, rStart, rEnd, rID;
	int32_t leftID, rightID, leftBrotherID;
	
	//Father ID
	int32_t fatherID = xGetID(iStart, iEnd);

	//mid = ((iEnd - iStart + 1) + iStart) >> 1;
	lStart = iStart;
	lEnd = iStart + ((iEnd - iStart) >> 1);
	leftID = xGetID(lStart, lEnd);
	leftBrotherID = ((lStart > 0) ? xGetID(lStart - (lEnd - lStart + 1), lEnd - (lEnd - lStart + 1)) : NONE);

	rStart = lEnd + 1;
	rEnd = iEnd;
	rightID = xGetID(rStart, rEnd);
	
	//Print left and right
	//std::cout << leftID  << " : (" << lStart << " , " << lEnd << " )" << std::endl;
	//std::cout << rightID << " : (" << rStart << " , " << rEnd << " )" << std::endl;

	//Create left struct
	sLeft.iStart = lStart;
	sLeft.iEnd = lEnd;
	sLeft.myID = leftID;
	sLeft.fatherID = fatherID;
	sLeft.leftBrotherID = leftBrotherID;
	sLeft.isRightBrother = false;

	//Create right struct
	sRight.iStart = rStart;
	sRight.iEnd = rEnd;
	sRight.myID = rightID;
	sRight.fatherID = fatherID;
	sRight.leftBrotherID = leftID;
	sRight.isRightBrother = true;

	this->silhouette_list.push_back(sLeft);
	this->silhouette_list.push_back(sRight);

	if (lEnd > lStart)
	{
		xCreateDyadicDecomposition(lStart, lEnd);
		xCreateDyadicDecomposition(rStart, rEnd);
	}
}

//This function returns a unique ID given a range.
//This is NOT NECESSARILY the same id as it is traversed in the silhouette decomposition
//However, it is GUARANTEED that each id is unique. 
int32_t gpcc::SilhouetteDecomposition::xGetID(uint32_t iStart, uint32_t iEnd)
{
	int32_t nImagesThisDepth = iEnd - iStart + 1;
	uint32_t depth = (uint32_t) log2((1 << this->nBits) / nImagesThisDepth);

	int32_t id = (1 << (depth)) + (iStart / nImagesThisDepth) - 1;
	return id;
}