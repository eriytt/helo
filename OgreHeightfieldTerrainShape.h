#ifndef OGREHEIGHTFIELDTERRAINSHAPE_H
#define OGREHEIGHTFIELDTERRAINSHAPE_H

#include <btBulletDynamicsCommon.h>

class OgreHeightfieldTerrainShape : public btHeightfieldTerrainShape
{
public:
  OgreHeightfieldTerrainShape(int heightStickWidth,int heightStickLength, void* heightfieldData,
			      btScalar minHeight, btScalar maxHeight, int upAxis) :
    btHeightfieldTerrainShape(heightStickWidth, heightStickLength, heightfieldData, 1.0,
                              minHeight, maxHeight, upAxis, PHY_FLOAT, false) {}

protected:
  btScalar getRawHeightFieldValue(int x, int y) const
  {
    // Ogre has the y axis reversed compared to the default Bullet
    // implementation
    unsigned int idx = ((m_heightStickLength - y - 1)
			* m_heightStickWidth) + x;
    // Always floats in Ogre
    return btScalar(m_heightfieldDataFloat[idx]);
  }


public:
  virtual void	processAllTriangles(btTriangleCallback* callback,
                                    const btVector3& aabbMin,
                                    const btVector3& aabbMax) const
  {
    // scale down the input aabb's so they are in local
    // (non-scaled) coordinates
    btVector3	localAabbMin = aabbMin * btVector3(1.f / m_localScaling[0],
                                                   1.f / m_localScaling[1],
                                                   1.f / m_localScaling[2]);
    btVector3	localAabbMax = aabbMax * btVector3(1.f / m_localScaling[0],
                                                   1.f / m_localScaling[1],
                                                   1.f / m_localScaling[2]);
    // account for local origin
    localAabbMin += m_localOrigin;
    localAabbMax += m_localOrigin;

    //quantize the aabbMin and aabbMax, and adjust the start/end ranges
    int quantizedAabbMin[3];
    int quantizedAabbMax[3];
    quantizeWithClamp(quantizedAabbMin, localAabbMin, 0);
    quantizeWithClamp(quantizedAabbMax, localAabbMax, 1);

    // expand the min/max quantized values. This is to catch the case
    // where the input aabb falls between grid points!
    // TODO: Is it really necessary to expand the min values?
    for (int i = 0; i < 3; ++i) {
      quantizedAabbMin[i]--;
      quantizedAabbMax[i]++;
    }

    int startX = 0;
    int endX = m_heightStickWidth - 1;
    int startJ = 0;
    int endJ = m_heightStickLength - 1;

    switch (m_upAxis)
      {
      case 0:
        if (quantizedAabbMin[1] > startX)
          startX = quantizedAabbMin[1];
        if (quantizedAabbMax[1] < endX)
          endX = quantizedAabbMax[1];
        if (quantizedAabbMin[2] > startJ)
          startJ = quantizedAabbMin[2];
        if (quantizedAabbMax[2] < endJ)
          endJ = quantizedAabbMax[2];
        break;
      case 1:
        if (quantizedAabbMin[0] > startX)
          startX = quantizedAabbMin[0];
        if (quantizedAabbMax[0] < endX)
          endX = quantizedAabbMax[0];
        if (quantizedAabbMin[2] > startJ)
          startJ = quantizedAabbMin[2];
        if (quantizedAabbMax[2] < endJ)
          endJ = quantizedAabbMax[2];
        break;
      case 2:
        if (quantizedAabbMin[0] > startX)
          startX = quantizedAabbMin[0];
        if (quantizedAabbMax[0] < endX)
          endX = quantizedAabbMax[0];
        if (quantizedAabbMin[1] > startJ)
          startJ = quantizedAabbMin[1];
        if (quantizedAabbMax[1] < endJ)
          endJ = quantizedAabbMax[1];
        break;
      default:
        // not a valid up axis
        btAssert(0);
      }

    for(int j = startJ; j < endJ; j++)
      {
        for(int x = startX; x < endX; x++)
	  {

	    /* We need to fill our "vertices" vector array with the correct vertex data
	     * for each square. "vertices" is an array of 5 coordinates where the first
	     * 3 are passed to the callback as the first triangle and the last three are
	     * passed as the second triangle. First and lass vertex is always the same.
	     * Ogre tesselates the terrain into triangles differently if the "row"
	     * (X coordinate) is even or odd, so we need to figure out which vertex should
	     * go where in the "vertices" array.
	     *
             *         	  |       | passed as the first triangle
	     * vertices = [w, x, y, z, w]
             *                  |       | passed as the second triangle
	     *
	     *     odd  -------------
	     *          |/|/|/|/|/|/|
	     *     even -------------
	     *          |\|\|\|\|\|\|
	     * ^   odd  1-2----------
	     * |        |/|/|/|/|/|/|
	     * X   even 0-3----------
	     *   Y ->
	     *
	     * If we enumerate the vertices like above, they need to go in the vertices array
	     * in the following order for an even row:
	     *
	     *   vertices = [0, 1, 2, 3, 0]
	     *
	     * For an odd row, the vertices are enumerated in the same way (see figure below),
	     * but put in "vertices" in a different order:
	     *
	     *   vertices = [1, 2, 3, 0, 1]
	     *
	     *     odd  -------------
	     *          |/|/|/|/|/|/|
	     *     even 1-2----------
	     *          |\|\|\|\|\|\|
	     * ^   odd  0-3---------
	     * |        |/|/|/|/|/|/|
	     * X   even -------------
	     *   Y ->
	     *
	     */

	    unsigned char idx_list[4];
	    btVector3 vertices[5];
	    // Determine index order
	    if (not (j & 1)) // even row
	      {
		idx_list[0] = 0;
		idx_list[1] = 1;
		idx_list[2] = 2;
		idx_list[3] = 3;
	      }
	    else // odd row
	      {
		idx_list[0] = 1;
		idx_list[1] = 2;
		idx_list[2] = 3;
		idx_list[3] = 0;
	      }

	    // Populate the "vertices" array according to the index order
	    getVertex(x    , j    , vertices[idx_list[0]]);  // Vertex 0
	    getVertex(x    , j + 1, vertices[idx_list[1]]);  // Vertex 1
	    getVertex(x + 1, j + 1, vertices[idx_list[2]]);  // Vertex 2
	    getVertex(x + 1, j    , vertices[idx_list[3]]);  // Vertex 3

	    // First and last vertex is the same
	    vertices[4] = vertices[0];

	    // Send the first 3 vertices as the first triangle
	    callback->processTriangle(vertices, x, j);
	    // Send the last 3 vertices as the second triangle
	    callback->processTriangle(&vertices[2], x, j);
	  }
      }
  }

  virtual const char*	getName()const {return "OGRE_HEIGHTFIELD";}

};

#endif // OGREHEIGHTFIELDTERRAINSHAPE_H
