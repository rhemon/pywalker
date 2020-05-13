
import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)
import math
import numpy as np
import Box2D  # The main library
# Box2D.b2 maps Box2D.b2Vec2 to vec2 (and so on)
from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody, kinematicBody, contactListener)

DEGTORAD = math.pi / 180

# --- constants ---
# Box2D deals with meters, but we want to display pixels,
# so define a conversion factor:
PPM = 10.0  # pixels per meter
TARGET_FPS = 20
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

# --- pygame setup ---
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Simple pygame example')
clock = pygame.time.Clock()

# --- pybox2d world setup ---
# Create the world
world = world(gravity=(0, -10), doSleep=True)

# And a static body to hold the ground shape
ground_body = world.CreateStaticBody(
    position=(0, 0),
    shapes=polygonShape(box=(50, 1)),
)

BODY_DIM = (2, 1)
BODY_POS = (10, 11)

LEG_INNER = 0.25
LEG_DIM = (0.25, 2)
LEGA_POS = (BODY_POS[0]-BODY_DIM[0]+LEG_DIM[0], BODY_POS[1]-BODY_DIM[1]+LEG_INNER)
LEGB_POS = (BODY_POS[0]+BODY_DIM[0]-LEG_DIM[0], BODY_POS[1]-BODY_DIM[1]+LEG_INNER)

boxbody = world.CreateDynamicBody(position=BODY_POS, angle=0)
box = boxbody.CreatePolygonFixture(box=BODY_DIM, density=1, friction=0.3)

legabody = world.CreateDynamicBody(position=LEGA_POS, angle=(-45/2)*DEGTORAD)
lega = legabody.CreatePolygonFixture(box=LEG_DIM, density=1, friction=0.3)
lowerlegabody = world.CreateDynamicBody(position=(LEGA_POS[0], LEGA_POS[1]-LEG_DIM[1]))
lowerlega = lowerlegabody.CreatePolygonFixture(box=LEG_DIM, density=1, friction=0.3)

legbbody = world.CreateDynamicBody(position=LEGB_POS, angle=(+45/2)*DEGTORAD)
legb = legbbody.CreatePolygonFixture(box=LEG_DIM, density=1, friction=0.3)
lowerlegbbody = world.CreateDynamicBody(position=(LEGB_POS[0], LEGB_POS[1]-LEG_DIM[1]))
lowerlegb = lowerlegbbody.CreatePolygonFixture(box=LEG_DIM, density=1, friction=0.3)

# legalowerbody = world.CreateDynamicBody(position=(11.5, 8))
# legalower = legalowerbody.CreatePolygonFixture(box=(0.25, 2), density=1, friction=0.3)

# legblowerbody = world.CreateDynamicBody(position=(8.5, 8))
# legblower = legblowerbody.CreatePolygonFixture(box=(0.25, 2), density=1, friction=0.3)

print(boxbody.worldCenter)

upperjointa = world.CreateRevoluteJoint(
    bodyA=boxbody,
    bodyB=legabody,
    # anchor=(LEGA_POS[0], LEGA_POS[0]-LEG_INNER/2)
    localAnchorA=(0-BODY_DIM[0]+LEG_DIM[0], -BODY_DIM[1]+LEG_INNER/2),
    localAnchorB=(0, LEG_DIM[1]-LEG_INNER/2),
    lowerAngle=(-25/2)*DEGTORAD,
    upperAngle=(+25/2)*DEGTORAD,
    enableLimit=True,
    enableMotor=True,
    maxMotorTorque=80,
    collideConnected=False
    # angle=(-10/2)*DEGTORAD
)
lowerjointa = world.CreateRevoluteJoint(
    bodyA=legabody,
    bodyB=lowerlegabody,
    # anchor=(LEGA_POS[0], LEGA_POS[0]-LEG_INNER/2)
    localAnchorA=(0, -LEG_DIM[1]+LEG_INNER/2),
    localAnchorB=(0, LEG_DIM[1]-LEG_INNER/2),
    lowerAngle=(-25/2)*DEGTORAD,
    upperAngle=(+25/2)*DEGTORAD,
    enableLimit=True,
    enableMotor=True,
    maxMotorTorque=80,
    collideConnected=False
    # angle=(-10/2)*DEGTORAD
)
upperjointb = world.CreateRevoluteJoint(
    bodyA=boxbody,
    bodyB=legbbody,
    localAnchorA=(0+BODY_DIM[0]-LEG_DIM[0], -BODY_DIM[1]+LEG_INNER/2),
    localAnchorB=(0, LEG_DIM[1]-LEG_INNER/2),
    lowerAngle=(-25/2)*DEGTORAD,
    upperAngle=(+25/2)*DEGTORAD,
    enableLimit=True,
    enableMotor=True,
    maxMotorTorque=80,
    collideConnected=False
    # motorSpeed = +1
)
lowerjointb = world.CreateRevoluteJoint(
    bodyA=legbbody,
    bodyB=lowerlegbbody,
    # anchor=(LEGA_POS[0], LEGA_POS[0]-LEG_INNER/2)
    localAnchorA=(0, -LEG_DIM[1]+LEG_INNER/2),
    localAnchorB=(0, LEG_DIM[1]-LEG_INNER/2),
    lowerAngle=(-25/2)*DEGTORAD,
    upperAngle=(+25/2)*DEGTORAD,
    enableLimit=True,
    enableMotor=True,
    maxMotorTorque=80,
    collideConnected=False
    # motorSpeed = +1,
    # angle=(-10/2)*DEGTORAD
)

colors = {
    staticBody: (255, 255, 255, 255),
    dynamicBody: (127, 127, 127, 255),
    kinematicBody: (255, 127, 127, 255),
}

gameOver = False
class ContactDetector(contactListener):
    def __init__(self):
        contactListener.__init__(self)

    def BeginContact(self, contact):
        global boxbody, gameOver
        contact_bodies = [contact.fixtureA.body,contact.fixtureB.body]
        if boxbody in contact_bodies and ground_body in contact_bodies:
            print("here")
            gameOver = True
    def EndContact(self, contact):
        pass

world.contactListener = ContactDetector()
# Let's play with extending the shape classes to draw for us.


def my_draw_polygon(polygon, body, fixture):
    vertices = [(body.transform * v) * PPM for v in polygon.vertices]
    # print(vertices)
    vertices = [(v[0], SCREEN_HEIGHT - v[1]) for v in vertices]
    # print(vertices)
    pygame.draw.polygon(screen, colors[body.type], vertices)
polygonShape.draw = my_draw_polygon


def my_draw_circle(circle, body, fixture):
    position = body.transform * circle.pos * PPM
    position = (position[0], SCREEN_HEIGHT - position[1])
    pygame.draw.circle(screen, colors[body.type], [int(
        x) for x in position], int(circle.radius * PPM))
    # Note: Python 3.x will enforce that pygame get the integers it requests,
    #       and it will not convert from float.
circleShape.draw = my_draw_circle

# --- main game loop ---

running = True
c = 0
while running and not gameOver:
    # Check the event queue
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            # The user closed the window or pressed escape
            running = False

    screen.fill((0, 0, 0, 0))
    # Draw the world
    for body in world.bodies:
        for fixture in body.fixtures:
            fixture.shape.draw(body, fixture)
    # if c == 20:
    upperjointa.motorSpeed = 4.0 * np.sign(np.random.randint(-1, 2))
    upperjointa.maxMotorTorque = 80*np.random.random()
    upperjointb.motorSpeed = 4.0 * np.sign(np.random.randint(-1, 2))
    upperjointb.maxMotorTorque = 80*np.random.random()
    lowerjointa.motorSpeed = 7.0 * np.sign(np.random.randint(-1, 2))
    lowerjointa.maxMotorTorque = 80*np.random.random()
    lowerjointb.motorSpeed = 7.0 * np.sign(np.random.randint(-1, 2))
    lowerjointb.maxMotorTorque = 80*np.random.random()
    # print(upperjointa.motorSpeed, upperjointb.motorSpeed, lowerjointa.motorSpeed, lowerjointb.motorSpeed)
    
    world.Step(TIME_STEP, 10, 10)
    
    # Flip the screen and try to keep at the target FPS
    pygame.display.flip()
    clock.tick(TARGET_FPS)

pygame.quit()
print('Done!') 