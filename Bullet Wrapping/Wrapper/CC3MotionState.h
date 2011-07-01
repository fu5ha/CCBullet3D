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

#ifndef CC3MOTIONSTATE_H_
#define CC3MOTIONSTATE_H_

#include "btTransform.h"
#include "btMotionState.h"

@class CC3Node;

/*
 * The CC3MotionState inherits from btMotionState and is passed in the constructor of a btRigidBody.
 * The CC3MotionState constructor takes an CC3Node: this node corresponds to the rendered equivalent
 * to the btRigidBody. This provides the bridging between the btRigidBody and the CC3Node, allowing the 
 * transformation from one to be passed to the other. During the simulation step of Bullet, the btMotionState
 * allows direct access to the transformation of the physics object to update any graphical peers.
 */
class CC3MotionState : public btMotionState {

public :
	CC3MotionState(CC3Node * sceneNode);
	virtual ~CC3MotionState();

	/**
	 * Gets the transformation of the the CC3Node and applies it to the btRigidBody.
	 * Called internally by the Bullet physics engine.
	 */
	virtual void getWorldTransform(btTransform& centerOfMassWorldTrans ) const;

	/**
	 * Gets the transformation of the the btRigidBody and applies it to the CC3Node.
	 * Called internally by the Bullet physics engine.
	 */
	virtual void setWorldTransform(const btTransform& centerOfMassWorldTrans);


private :
	CC3Node * _node;
	
};


#endif /*CC3MOTIONSTATE_H_*/
