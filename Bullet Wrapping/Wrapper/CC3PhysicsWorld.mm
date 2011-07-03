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

#import "CC3PhysicsWorld.h"
#import "CC3PhysicsObject3D.h"
#import "cocos2d.h"


@implementation CC3PhysicsWorld
@synthesize _discreteDynamicsWorld;

- (id) init {
    if ((self = [super init])) 
	{
    	
    	_lastStepTime = [[NSDate alloc] init];
       	_physicsObjects = [[NSMutableArray alloc] init];
        broadphase = new btDbvtBroadphase();
		collisionConfiguration = new btDefaultCollisionConfiguration();
		dispatcher = new btCollisionDispatcher(collisionConfiguration);
		solver = new btSequentialImpulseConstraintSolver();
		dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
		[self setDiscreteDynamicsWorld:dynamicsWorld];
        _collidingObjects = [[NSMutableArray alloc] init];
        _thisCollidingObjects = [[NSMutableArray alloc] init];
    }
	
    return self;
}

- (void) dealloc 
{
	
	[_lastStepTime release];
	[_physicsObjects release];
    [_collidingObjects release];
    delete broadphase;
    delete dynamicsWorld;
    delete collisionConfiguration;
    delete dispatcher;
    delete solver;

	[super dealloc];
}

- (void) setDiscreteDynamicsWorld:(btDiscreteDynamicsWorld *)discreteDynamicsWorld {
	_discreteDynamicsWorld = discreteDynamicsWorld;
}

- (void) addPhysicsObject:(CC3PhysicsObject3D *)physicsObject {
	
	// Add collision object to dynamics world
	_discreteDynamicsWorld->addRigidBody(physicsObject.rigidBody);
	
	// Add to physics list
    if (!physicsObject.isStatic) {
        [_physicsObjects addObject:physicsObject];
    }
	
}
- (void) removePhysicsObject:(CC3PhysicsObject3D *)physicsObject 
{
	// Remove from render list
	[physicsObject.node remove];

	// Remove collision object from dynamics world
	_discreteDynamicsWorld->removeRigidBody(physicsObject.rigidBody);
	
	// Remove from physics list
	[_physicsObjects removeObject:physicsObject];
}

- (void) removeAllChildren			
{
	[super  removeAllChildren];
	
	for (CC3PhysicsObject3D * physicsObject in _physicsObjects) {
		_discreteDynamicsWorld->removeRigidBody(physicsObject.rigidBody);
	}
	
	[_physicsObjects removeAllObjects];
}

- (void) synchTransformation {
	// Get time since last step
	NSDate * currentTime = [[NSDate alloc] init];
	
	NSTimeInterval timeInterval = [currentTime timeIntervalSinceDate:_lastStepTime];
//	float optimalInterval = 1. / 60.;
	
//	if (timeInterval > optimalInterval) {
//		timeInterval = optimalInterval;
//	}
	
	// Update the simulation
	_discreteDynamicsWorld->stepSimulation(timeInterval, 2);
	[_lastStepTime release];
	_lastStepTime = currentTime;

	// Update all global matrices
	for (CC3PhysicsObject3D *object in _physicsObjects) {
        btTransform gTrans;
        object.rigidBody->getMotionState()->getWorldTransform(gTrans);
        btVector3 gPos = gTrans.getOrigin();
        btVector3 axis = gTrans.getRotation().getAxis();
        float angle = gTrans.getRotation().getAngle();
        CC3Vector4 quaternion;
        float sinAngle;
        angle *= 0.5f;
        axis = axis.normalized();
        sinAngle = sin(angle);
        quaternion.x = (axis.getX() * sinAngle);
        quaternion.y = (axis.getY() * sinAngle);
        quaternion.z = (axis.getZ() * sinAngle);
        quaternion.w = cos(angle);
        object.node.location = CC3VectorMake(gPos.getX(), gPos.getY(), gPos.getZ());
        object.node.quaternion = quaternion;
    }
    int numManifolds = _discreteDynamicsWorld->getDispatcher()->getNumManifolds();
	
    for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold =  _discreteDynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		btRigidBody* obA = static_cast<btRigidBody*>(contactManifold->getBody0());
		btRigidBody* obB = static_cast<btRigidBody*>(contactManifold->getBody1());
        for (CC3PhysicsObject3D * object in _physicsObjects) {
            if (obA == object.rigidBody or obB == object.rigidBody) {
                [_thisCollidingObjects addObject:object];
                if (object.collidingCount > 0) {
                } else {
                    [_collidingObjects addObject:object];
                    object.colliding = YES;
                    object.collidingCount = 1;
                }
                if (obA == object.rigidBody) {
                    object.collidingWith = obB;
                } else {
                    object.collidingWith = obA;
                }
            }
        }
	}
    NSMutableArray *objectsToDelete = [[[NSMutableArray alloc] init] autorelease];
    for (CC3PhysicsObject3D *object in _collidingObjects) {
        if ([_thisCollidingObjects containsObject:object]) {
        } else {
            object.colliding = NO;
            object.collidingWith = nil;
            object.collisionPhase = @"ended";
            object.collidingCount = 0;
        }
    }
    for (CC3PhysicsObject3D *object in objectsToDelete) {
        [_collidingObjects removeObject:object];
    }
    [_thisCollidingObjects removeAllObjects];
}

- (void) setGravity:(float)x y:(float)y z:(float)z {
	_discreteDynamicsWorld->setGravity(btVector3(x, y, z));
}

- (NSMutableArray *) getCollidingObjects
{
    return _collidingObjects;
}

- (CC3PhysicsObject3D *) createPhysicsObject:(CC3Node *)node shape:(btCollisionShape *)shape mass:(float)mass restitution:(float)restitution position:(CC3Vector)position {
	// Create a motion state for the object
	btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(position.x, position.y, position.z)));
	
	// Create a rigid body
	btVector3 localInertia(0, 0, 0);
	shape->calculateLocalInertia(mass, localInertia);
	btRigidBody * rigidBody = new btRigidBody(mass, motionState, shape, localInertia);
	rigidBody->setRestitution(restitution);
	rigidBody->setActivationState(DISABLE_DEACTIVATION);

	// Create a physics object and add it to the physics world
    if (mass > 0) {
        isstatic = NO;
    } else {
        isstatic = YES;
	}
    CC3PhysicsObject3D *physicsObject = [[CC3PhysicsObject3D alloc] initWithNode:node andRigidBody:rigidBody isStatic:isstatic];
    [self addPhysicsObject:physicsObject];
	return physicsObject;
}

@end
