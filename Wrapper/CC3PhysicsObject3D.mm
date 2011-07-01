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
};

#import "CC3PhysicsObject3D.h"
#import "CC3Node.h"


#import "btBulletDynamicsCommon.h"

@implementation CC3PhysicsObject3D

@synthesize node = _node;
@synthesize rigidBody = _rigidBody;

- (id) initWithNode:(CC3Node *)node andRigidBody:(btRigidBody *)rigidBody {
    if ((self = [super init])) {
    	_node = [node retain];
    	_rigidBody = rigidBody;
    	
    }
	
    return self;
}

- (void) dealloc {
	[_node release];
	
	delete _rigidBody->getMotionState();
	delete _rigidBody->getCollisionShape();
	delete _rigidBody;

	[super dealloc];
}

- (void) applyForce:(CC3Vector)force withPosition:(CC3Vector)position {
	btVector3 bodyForce(force.x, force.y, force.z);
	btVector3 bodyPosition(position.x, position.y, position.z);

	_rigidBody->applyForce(bodyForce, bodyPosition);	
}

- (void) applyImpulse:(CC3Vector)impulse withPosition:(CC3Vector)position {
	btVector3 bodyImpulse(impulse.x, impulse.y, impulse.z);
	btVector3 bodyPosition(position.x, position.y, position.z);
	
	_rigidBody->applyImpulse(bodyImpulse, bodyPosition);
}

- (CC3Vector) getGlobalPosition {
	btTransform gTrans;
	_rigidBody->getMotionState()->getWorldTransform(gTrans);
	CC3Vector position = cc3v(gTrans.getOrigin().getX(), gTrans.getOrigin().getY(), gTrans.getOrigin().getZ());
	return position;
}

- (void) setGlobalPosition:(CC3Vector)position  {
	btTransform gTrans = btTransform(btQuaternion(0,0,0,1),btVector3(position.x, position.y, position.z));
	_rigidBody->getMotionState()->setWorldTransform(gTrans);
}

@end
