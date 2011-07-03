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



#import <Foundation/Foundation.h>
#import "btBulletDynamicsCommon.h"
#import "CC3PhysicsWorld.h"

@class CC3Node;
class btRigidBody;

/**
 * The CC3PhysicsObject3D contains both an CC3Node and a btRigidBody providing a strong link between both
 * the physicaly object and its rendered peer.
 * 
 * Note that when the CC3PhysicsObject3D is deleted, the associated btRigidBody, the btCollisionShape and
 * the btMotionState are also deleted. During construction, The CC3Node is retained and when the
 * CC3PhysicsObject3D is deleted the CC3Node is released.
 */
@interface CC3PhysicsObject3D : NSObject {
	
@private
	CC3Node * _node;
	btRigidBody * _rigidBody;
    btCollisionShape *_shape;
    btPoint2PointConstraint *p2p;
    BOOL isStatic;
    BOOL colliding;
    int collidingCount;
    NSString *collisionPhase;
    btRigidBody * collidingWith;

}

/**
 * Returns the associated CC3Node.
 */
@property (readonly) CC3Node * node;

/**
 * Returns the associated btRigidBody.
 */
@property (readonly) btRigidBody * rigidBody;

/**
 * Returns the associated btCollisionShape
 */
@property (readonly) btCollisionShape * shape;
/**
 * Returns YES if an object is static
 */
@property (readonly) BOOL isStatic;

@property (nonatomic, assign) BOOL colliding;

@property (nonatomic, assign) btRigidBody * collidingWith;

@property (nonatomic, assign) NSString * collisionPhase;

@property (nonatomic, assign) int collidingCount;

/**
 * Initialises the CC3PhysicsObject3D with an CC3Node and a btRigidBody.
 * @param node The CC3Node.
 * @param rigidBody The btRigidBody.
 */
- (id) initWithNode:(CC3Node *)node andRigidBody:(btRigidBody *)rigidBody isStatic:(BOOL)isstatic;

/**
 * Applies a force, defined as a vector, to the btRigidBody at a given vector position.
 * @param force The force to be applied.
 * @param position The position at which the force is applied.
 */
- (void) applyForce:(CC3Vector)force withPosition:(CC3Vector)position;

/**
 * Applies an impulse, defined as a vector, to the btRigidBody at a given vector position.
 * @param force The force to be applied.
 * @param position The position at which the force is applied.
 */
- (void) applyImpulse:(CC3Vector)force withPosition:(CC3Vector)position;

@end
