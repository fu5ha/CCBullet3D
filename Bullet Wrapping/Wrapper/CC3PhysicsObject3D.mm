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
#import "CC3PhysicsWorld.h"
#import "CC3Node.h"

#import "btBulletDynamicsCommon.h"

@implementation CC3PhysicsObject3D

@synthesize node = _node;
@synthesize rigidBody = _rigidBody;
@synthesize shape = _shape;
@synthesize isStatic;
@synthesize colliding;
@synthesize collidingWith;
@synthesize collisionPhase;
@synthesize collidingCount;

- (id) initWithNode:(CC3Node *)node andRigidBody:(btRigidBody *)rigidBody isStatic:(BOOL)isstatic {
    if ((self = [super init])) {
    	_node = [node retain];
    	_rigidBody = rigidBody;
        _shape = _rigidBody->getCollisionShape();
        isStatic = isstatic;
        colliding = NO;
        collidingWith = nil;
    	collisionPhase = nil;
    }
	
    return self;
}

- (void) dealloc {
	[_node release];
	
	delete _rigidBody->getMotionState();
	delete _shape;
	delete _rigidBody;
    delete p2p;
	[super dealloc];
}

- (void) applyForce:(CC3Vector)force withPosition:(CC3Vector)position {
	btVector3 bodyForce(force.x, force.y, force.z);
	btVector3 bodyPosition(position.x, position.y, position.z);

	_rigidBody->applyForce(bodyForce, bodyPosition);	
}

-(void) applyImpulse:(CC3Vector)force withPosition:(CC3Vector)position {
    btVector3 bodyForce(force.x, force.y, force.z);
	btVector3 bodyPosition(position.x, position.y, position.z);
    
	_rigidBody->applyImpulse(bodyForce, bodyPosition);
}

- (void) setObjectLocation:(CC3Vector)position world:(CC3PhysicsWorld *)world {
    btVector3 btPosition = btVector3(position.x, position.y, position.z);
    p2p = new btPoint2PointConstraint(*_rigidBody, btVector3(_node.location.x, _node.location.y, _node.location.z));
    world._discreteDynamicsWorld->addConstraint(p2p);
    p2p->m_setting.m_impulseClamp = 30;
    //very weak constraint for picking
    p2p->m_setting.m_tau = 0.001f;
    p2p->setPivotB(btPosition);
}

- (void) setRotationQuaternion:(CC3Vector4)quaternion{
    btTransform nTrans;
    _rigidBody->getMotionState()->getWorldTransform(nTrans);
    btTransform rTrans;
    rTrans = btTransform(btQuaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w), nTrans.getOrigin());
    _rigidBody->getMotionState()->setWorldTransform(rTrans);
}

@end
