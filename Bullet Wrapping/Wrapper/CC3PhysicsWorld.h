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

#import "CC3Node.h"
#import <Foundation/Foundation.h>
#import "btBulletDynamicsCommon.h"

@class CC3PhysicsObject3D;
class btRigidBody;
class btDiscreteDynamicsWorld;
class btCollisionShape;

/**
 * The CC3PhysicsWorld provides a wrapper to the btDiscreteDynamicsWorld and contains all the CC3PhysicsObject3D objects. 
 * It inherits from CC3Node so is added directly to the scene. At every frame it updates automatically the 
 * btDiscreteDynamicsWorld (the physics simulation is updated) and hence the transformations of the physics
 * objects (btRigidBody) are updated.
 */
@interface CC3PhysicsWorld : CC3Node {
	
@private
	btDiscreteDynamicsWorld * _discreteDynamicsWorld;
    btDiscreteDynamicsWorld * dynamicsWorld;
    btBroadphaseInterface *broadphase;
    btDefaultCollisionConfiguration *collisionConfiguration;
    btSequentialImpulseConstraintSolver *solver;
    btCollisionDispatcher *dispatcher;
    
	NSDate * _lastStepTime;
	NSMutableArray * _physicsObjects;
    NSMutableArray * _collidingObjects;
    CC3PhysicsObject3D *_collisionObject1;
    CC3PhysicsObject3D *_collisionObject2;
    
    BOOL isstatic;

}

@property (readonly) btDiscreteDynamicsWorld * _discreteDynamicsWorld;

/**
 * Initialises the CC3PhysicsWorld;
 */
- (id) init;

/**
 * Sets the btDiscreteDynamicsWorld in which the Bullet Physics simulation takes place. The btDiscreteDynamicsWorld is
 * automatically stepped at every rendered frame.
 */
- (void) setDiscreteDynamicsWorld:(btDiscreteDynamicsWorld *)discreteDynamicsWorld;

/**
 *steps physics simulation and updates meshes to match the physics simulation. YOU NEED TO CALL THIS IN AN
 *UPDATE METHOD!
 */
-(void) synchTransformation;

/**
 * Adds a new CC3PhysicsObject3D containing both a btRigidBody and an CC3Node. Note, this DOES NOT add the
 * node contained in the physics object to the scene: this needs to be done independently. The btRigidBody is
 * added to the btDiscreteDynamicsWorld.
 * @param physicsObject The CC3PhysicsObject3D containing both btRigidBody and CC3Node.
 */
- (void) addPhysicsObject:(CC3PhysicsObject3D *)physicsObject;

/**
 * Removes an CC3PhysicsObject3D from the physics world. Note, this DOES remove the associated CC3Node
 * from its parent node in the scene. The btRigidBody is removed from the btDiscreteDynamicsWorld.
 * @param physicsObject The CC3PhysicsObject3D to remove.
 */
- (void) removePhysicsObject:(CC3PhysicsObject3D *)physicsObject;

/**
 * Sets the size and direction of gravity in the physics simulation.
 * @param x The x component of the gravity vectory.
 * @param y The y component of the gravity vectory.
 * @param z The z component of the gravity vectory.
 */
- (void) setGravity:(float)x y:(float)y z:(float)z;

/**
 * Utility method to create an CC3PhysicsObject3D from an CC3Node and a btCollisionShape. A btRigidBody is created for
 * the shape with an CC3MotionShape containing the node. Both btRigidBody and CC3Node are returned in the CC3PhysicsObject3D.
 * @param node The CC3Node to be manipulated as a result of the physics simulation.
 * @param shape The shape of the object.
 * @param mass The mass of the object.
 * @param restitution The restitution of the object.
 * @return (autorelease) The created CC3PhysicsObject3D containing the btRigidBody and the CC3Node.
 */
- (CC3PhysicsObject3D *) createPhysicsObject:(CC3Node *)node shape:(btCollisionShape *)shape mass:(float)mass restitution:(float)restitution position:(CC3Vector)position;

- (NSMutableArray *) getCollidingObjects;

@end
