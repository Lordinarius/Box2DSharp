using System.Numerics;

namespace Box2DSharp.Common
{
    public interface IDrawer
    {
        DrawFlag Flags { get; set; }

        /// Draw a closed polygon provided in CCW order.
        void DrawPolygon(V2[] vertices, int vertexCount, in Color color);

        /// Draw a solid closed polygon provided in CCW order.
        void DrawSolidPolygon(V2[] vertices, int vertexCount, in Color color);

        /// Draw a circle.
        void DrawCircle(in V2 center, F radius, in Color color);

        /// Draw a solid circle.
        void DrawSolidCircle(in V2 center, F radius, in V2 axis, in Color color);

        /// Draw a line segment.
        void DrawSegment(in V2 p1, in V2 p2, in Color color);

        /// Draw a transform. Choose your own length scale.
        /// @param xf a transform.
        void DrawTransform(in Transform xf);

        /// Draw a point.
        void DrawPoint(in V2 p, F size, in Color color);
    }
}