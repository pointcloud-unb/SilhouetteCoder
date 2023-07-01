#ifndef SILHOUETTEDECOMPOSITION_H
#define SILHOUETTEDECOMPOSITION_H

#include <utility>
#include <stdint.h>
#include <vector>

#define NONE -1

namespace gpcc {

	typedef struct Silhouette {
		uint32_t iStart;
		uint32_t iEnd;
		int32_t myID;
		int32_t fatherID;
		int32_t leftBrotherID;
		bool isRightBrother;				
	} silhouette;

	class SilhouetteDecomposition
	{
		private:
			uint32_t nBits;
			std::vector<Silhouette> silhouette_list;

			void xCreateDyadicDecomposition(uint32_t iStart, uint32_t iEnd);

			int32_t xGetID(uint32_t iStart, uint32_t iEnd);

			

		public:
			SilhouetteDecomposition();
			SilhouetteDecomposition(uint32_t nbits);
			
			void createDyadicDecomposition();
			void printList();

			int getNumberOfDecompositions() { return silhouette_list.size(); }
			Silhouette getSilhouette(uint32_t i) { return silhouette_list[i]; }

	};


}

#endif
