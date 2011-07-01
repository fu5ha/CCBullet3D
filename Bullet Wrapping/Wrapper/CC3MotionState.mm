/*
 * CC3: http://isgl3d.com
 *
 * Copyright (c) 2010-2011 Stuart Caunt
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

extern "C" {
#import "CC3Foundation.h"
}

#import "CC3MotionState.h"
#import "CC3Node.h"
#import "CC3GLMatrix.h"
#import "btVector3.h"
#import "btQuaternion.h"

CC3MotionState::CC3MotionState(CC3Node * node) :
	_node(node) {
}

CC3MotionState::~CC3MotionState() {
}

void CC3MotionState::getWorldTransform(btTransform& centerOfMassWorldTrans) const {
	float transformation[16];
	CC3GLMatrix *nodeTransform = [[CC3GLMatrix alloc] init];
    [nodeTransform populateFromTranslation:_node.location];
    [nodeTransform rotateByQuaternion:_node.quaternion];
	centerOfMassWorldTrans.setFromOpenGLMatrix(transformation);
}

void CC3MotionState::setWorldTransform(const btTransform& centerOfMassWorldTrans) {
    btVector3 rPoint;
    btQuaternion rRot;
	rPoint = btVector3(centerOfMassWorldTrans.getOrigin());
    rRot = btQuaternion(centerOfMassWorldTrans.getRotation());
    btVector3 rRotAxisBt = rRot.getAxis();
    CC3Vector rRotAxisCC3 = CC3VectorMake(rRotAxisBt.getX(), rRotAxisBt.getY(), rRotAxisBt.getZ());
    float rRotAngle = rRot.getAngle();
    _node.location = CC3VectorMake(rPoint.getX(), rPoint.getY(), rPoint.getZ());
    _node.quaternion = CC3Vector4Make(rRotAxisCC3.x, rRotAxisCC3.y, rRotAxisCC3.z, rRotAngle);
}