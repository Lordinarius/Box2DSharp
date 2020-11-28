using System;
using System.Linq;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using OpenToolkit.Windowing.Common;
using OpenToolkit.Windowing.Common.Input;
using Testbed.Basics;
using Vector2 = System.Numerics.Vector2;
using Color = Box2DSharp.Common.Color;

namespace Testbed.Tests
{
    [TestCase("Geometry", "Convex Hull")]
    public class ConvexHull : Test
    {
        private const int Count = Settings.MaxPolygonVertices;

        private V2[] _points = new V2[Settings.MaxPolygonVertices];

        private int _count;

        private bool _auto;

        public ConvexHull()
        {
            Generate();
            _auto = false;
        }

        /// <inheritdoc />
        public override void OnKeyDown(KeyboardKeyEventArgs key)
        {
            if (key.Key == Key.A)
            {
                _auto = !_auto;
            }

            if (key.Key == Key.G)
            {
                Generate();
            }
        }

        protected override void OnRender()
        {
            DrawString("Press g to generate a new random convex hull");
            DrawString("Press a to toggle random convex hull auto generation");
            var shape = new PolygonShape();
            shape.Set(_points);
            var drawLine = new V2[shape.Count + 1];
            Array.Copy(shape.Vertices.ToArray(), drawLine, shape.Count);
            drawLine[drawLine.Length - 1] = shape.Vertices[0];
            Drawer.DrawPolygon(drawLine, drawLine.Length, Color.FromArgb(0.9f, 0.9f, 0.9f));

            for (var i = 0; i < _count; ++i)
            {
                Drawer.DrawPoint(_points[i], 3.0f, Color.FromArgb(0.3f, 0.9f, 0.3f));
                Drawer.DrawString(_points[i] + new V2(0.05f, 0.05f), i.ToString());
            }

            Drawer.DrawPoint(V2.Zero, 5f, Color.Yellow);

            if (_auto && !TestSettings.Pause)
            {
                Generate();
            }
        }

        void Generate()
        {
            var lowerBound = new V2(-8.0f, -8.0f);
            var upperBound = new V2(8.0f, 8.0f);

            for (var i = 0; i < Count; ++i)
            {
                var x = 10.0f * RandomFloat();
                var y = 10.0f * RandomFloat();

                // Clamp onto a square to help create collinearities.
                // This will stress the convex hull algorithm.
                var v = new V2(x, y);
                v = V2.Clamp(v, lowerBound, upperBound);
                _points[i] = v;
            }

            _count = Count;
        }
    }
}