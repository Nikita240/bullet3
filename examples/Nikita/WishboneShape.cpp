#include "WishboneShape.h"

#include "btBulletDynamicsCommon.h"

#include "LinkShape.h"

WishboneShape::WishboneShape(
	const btVector3 a_knucklePoint,
	const btVector3 a_subframePointA,
	const btVector3 a_subframePointB)
{
	LinkShape* frontLinkShape = new LinkShape(a_knucklePoint, a_subframePointA);
	LinkShape* rearLinkShape = new LinkShape(a_knucklePoint, a_subframePointB);

	addChildShape(btTransform::getIdentity(), frontLinkShape);
	addChildShape(btTransform::getIdentity(), rearLinkShape);
}
