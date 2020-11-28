using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Basics;

namespace Testbed.Tests
{
    [TestCase("Examples", "Slider Crank 1")]
    public class SliderCrank1 : Test
    {
        public SliderCrank1()
        {
            Body ground;
            {
                var bd = new BodyDef {Position = new V2(0.0f, 17.0f)};
                ground = World.CreateBody(bd);
            }

            {
                var prevBody = ground;

                // Define crank.
                {
                    var shape = new PolygonShape();
                    shape.SetAsBox(4.0f, 1.0f);

                    var bd = new BodyDef {BodyType = BodyType.DynamicBody, Position = new V2(-8.0f, 20.0f)};
                    var body = World.CreateBody(bd);
                    body.CreateFixture(shape, 2.0f);

                    var rjd = new RevoluteJointDef();
                    rjd.Initialize(prevBody, body, new V2(-12.0f, 20.0f));
                    World.CreateJoint(rjd);

                    prevBody = body;
                }

                // Define connecting rod
                {
                    var shape = new PolygonShape();
                    shape.SetAsBox(8.0f, 1.0f);

                    var bd = new BodyDef {BodyType = BodyType.DynamicBody, Position = new V2(4.0f, 20.0f)};
                    var body = World.CreateBody(bd);
                    body.CreateFixture(shape, 2.0f);

                    var rjd = new RevoluteJointDef();
                    rjd.Initialize(prevBody, body, new V2(-4.0f, 20.0f));
                    World.CreateJoint(rjd);

                    prevBody = body;
                }

                // Define piston
                {
                    var shape = new PolygonShape();
                    shape.SetAsBox(3.0f, 3.0f);

                    var bd = new BodyDef
                    {
                        BodyType = BodyType.DynamicBody, FixedRotation = true,
                        Position = new V2(12.0f, 20.0f)
                    };
                    var body = World.CreateBody(bd);
                    body.CreateFixture(shape, 2.0f);

                    var rjd = new RevoluteJointDef();
                    rjd.Initialize(prevBody, body, new V2(12.0f, 20.0f));
                    World.CreateJoint(rjd);

                    var pjd = new PrismaticJointDef();
                    pjd.Initialize(ground, body, new V2(12.0f, 17.0f), new V2(1.0f, 0.0f));
                    World.CreateJoint(pjd);
                }
            }
        }
    }
}