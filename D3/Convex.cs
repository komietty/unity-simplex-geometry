using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine.Assertions;
using static Unity.Mathematics.math;

namespace kmty.geom.d3 {
    using f3 = float3;
    using d3 = double3;
    using SG = Segment;
    using TN = TriangleNode;

    public class Convex { 
        private IEnumerable<f3> outsides;
        public AABB aabb       { get; protected set; }
        public List<TN> nodes  { get; protected set; }
        public d3 centroid     { get; protected set; }
        public List<TN>           taggeds { get; protected set; } // node that will be removed
        public List<(TN n, SG s)> contour { get; protected set; } // node that will be connected
        double h = 1e-10d;

        public Convex(IEnumerable<f3> originals) {
            var xs = originals.OrderBy(p => p.x);
            var p0 = xs.First();
            var p1 = xs.Last();
            var p2 = originals.OrderByDescending(p => lengthsq(cross(p - p0, p1 - p0))).First();
            var p3 = originals.OrderByDescending(f => new TN(p0, p1, p2).DistFactor(f)).First();
            this.centroid = (p0 + p1 + p2 + p3) * 0.25f;

            var n0 = new TN(p0, p1, p2); n0.SetNormal(centroid);
            var n1 = new TN(p1, p2, p3); n1.SetNormal(centroid);
            var n2 = new TN(p2, p3, p0); n2.SetNormal(centroid);
            var n3 = new TN(p3, p0, p1); n3.SetNormal(centroid);
            n0.InitNeighbors(n1, n2, n3);
            n1.InitNeighbors(n2, n3, n0);
            n2.InitNeighbors(n3, n0, n1);
            n3.InitNeighbors(n0, n1, n2);

            this.nodes    = new List<TN>() { n0, n1, n2, n3 };
            this.outsides = originals;
            this.taggeds  = new List<TN>();
            this.contour  = new List<(TN n, SG s)>();
        }

        public void ExpandLoop(int maxitr = int.MaxValue) {
            int itr = 0;
            while (itr < maxitr) {
                UnityEngine.Debug.Log(itr);
                itr++;
                if (!Expand()) break;
            }
        }

        public bool Expand() {
            taggeds.Clear();
            contour.Clear();
            outsides = outsides.Where(p => !Contains(p));
            if (outsides.Count() == 0) return false;

            if (FindApex(out d3 apex, out TN root)) {

                // tag fase
                taggeds.Add(root);
                NeighborSearch(root.neibs[0], root, apex);
                NeighborSearch(root.neibs[1], root, apex);
                NeighborSearch(root.neibs[2], root, apex);
                //for (var i = 0; i < 3; i++) {
                //    NeighborSearch(root.GetFrNeibs(i), root, apex);
                //}

                // rmv fase
                UnityEngine.Debug.Log("tag count" + taggeds.Count);
                UnityEngine.Debug.Log("cnt count" + contour.Count);

                foreach (var n in taggeds) {
                    //for (var i = 0; i < n.NumOfNeibs(); i++) {
                    //    var nei = n.GetFrNeibs(i);
                    //    nei.RmvFrNeibs(n);
                    //}
                    for (var i = 0; i < 3; i++) {
                        UnityEngine.Debug.Log("f" + n.flags[i]);
                        if (n.flags[i]) {
                            var nei = n.neibs[i];
                            UnityEngine.Debug.Log(nei.HasInNeibs(n));
                            nei.RmvFrNeibs(n);
                        }
                    }
                    nodes.Remove(n);
                    RollbackCentroid(n);
                }

                // add fase
                var cone = new (TN n, d3 a, d3 b)[contour.Count];
                for (var i = 0; i < contour.Count; i++) {
                    var c = contour[i];
                    var s = c.s;
                    var pair = c.n;
                    var curr = new TN(apex, s.a, s.b);

                    foreach (var f in pair.flags) {
                        UnityEngine.Debug.Log("cnt_nei" + f);
                    }

                    curr.AddToNeibs(pair);
                    pair.AddToNeibs(curr);

                    foreach (var cf in cone) {
                        if (s.a.Equals(cf.a) || s.a.Equals(cf.b) || s.b.Equals(cf.a) || s.b.Equals(cf.b)) {
                            var sbln = cf.n;
                            if (!sbln.HasInNeibs(curr)) sbln.AddToNeibs(curr);
                            else throw new System.Exception();
                            if (!curr.HasInNeibs(sbln)) curr.AddToNeibs(sbln);
                            else throw new System.Exception();

                        }
                    }
                    cone[i] = (curr, s.a, s.b);
                    UpdateCentroid(curr);
                }

                this.nodes.AddRange(cone.Select(l => l.n));
                foreach (var n in nodes) n.SetNormal(centroid);
                return true;
            }
            return false;
        }

        bool FindApex(out d3 apex, out TN root) {
            foreach (var n in nodes) {
                var s = outsides.Where(p => dot(p - n.center, n.normal) > 0);
                if (s.Count() > 0) {
                    apex = s.OrderByDescending(p => n.DistFactor(p)).First();
                    root = n;
                    return true;
                }
            }
            apex = default;
            root = default;
            return false;
        }

        void NeighborSearch(TN curr, TN pair, d3 p) {
            if (dot(p - curr.center, curr.normal) > h) {
                taggeds.Add(curr);
                //for (var i = 0; i < curr.NumOfNeibs(); i++) {
                for (var i = 0; i < 3; i++) {
                    if (curr.flags[i]) {
                        var n = curr.neibs[i];
                        if (!taggeds.Contains(n)) NeighborSearch(n, curr, p);
                    }
                }
            } else {
                var t = (curr, curr.Common(pair));
                if (!contour.Contains(t)) contour.Add(t);
            }
        }

        public bool Contains(d3 p) {
            foreach (var n in nodes) {
                if (dot(p - n.center, n.normal) > h) return false;
            }
            return true;
        }

        void UpdateCentroid(TN n) {
            var c = nodes.Count + 1;
            var f = 1f / c;
            centroid *= (c - 1);
            centroid = f * (centroid + n.center);
        }

        void RollbackCentroid(TN n) {
            var c = nodes.Count + 1;
            var f = 1f / (c - 1);
            centroid *= c;
            centroid = f * (centroid - n.center);
        }

        public void Draw() {
            for (int i = 0; i < nodes.Count; i++) {
                var n = nodes[i];
                n.t.Draw();
            }
        }
    }

    public class TriangleNode : System.IEquatable<TN> {
        public Triangle t { get; }
        public d3 center { get; }
        public d3 normal { get; private set; }
        //List<TN> neighbors;
        public TN[]   neibs { get; private set; }
        public bool[] flags { get; private set; }

        public TriangleNode(d3 p1, d3 p2, d3 p3) {
            this.t = new Triangle(p1, p2, p3);
            this.center = t.GetGravityCenter();
            this.neibs = new TN[3]{ null, null, null };
            this.flags = new bool[3]{ false, false, false };
        }

        public void SetNormal(d3 centroid) {
            var v = cross(t.b - t.a, t.c - t.a);
            var s = dot(v, centroid - t.a) > 0 ? -1 : 1;
            this.normal = normalize(v) * s;
        }

        public void InitNeighbors(TN n0, TN n1, TN n2) {
            this.neibs[0] = n0;
            this.neibs[1] = n1;
            this.neibs[2] = n2;
            this.flags[0] = true;
            this.flags[1] = true;
            this.flags[2] = true;
        }

        public bool HasInNeibs(TN n) {
            for (int i = 0; i < 3; i++) {
                if (flags[i] && neibs[i].Equals(n)) { return true; }
                //if (flags[i] && neibs.Equals(n)) { return true; }
            }
            return false;
        }

        public bool AddToNeibs(TN n) {
            for(int i = 0; i < 3; i++) {
                if (!flags[i]) {
                    neibs[i] = n;
                    flags[i] = true;
                    return true;
                }
            }
            throw new System.Exception();
        }

        public bool RmvFrNeibs(TN n) {
            for(int i = 0; i < 3; i++) {
                if (flags[i] && neibs[i].Equals(n)) {
                    neibs[i] = null;
                    flags[i] = false;
                    return true;
                }
            }
            throw new System.Exception();
        }

        public double DistFactor(d3 p) => lengthsq(cross(p - t.a, dot(p - t.b, p - t.c)));

        public SG Common(TN pair) {
            if (this.t.HasVert(pair.t.a) && this.t.HasVert(pair.t.b)) return new SG(pair.t.a, pair.t.b); 
            if (this.t.HasVert(pair.t.b) && this.t.HasVert(pair.t.c)) return new SG(pair.t.b, pair.t.c); 
            if (this.t.HasVert(pair.t.c) && this.t.HasVert(pair.t.a)) return new SG(pair.t.c, pair.t.a);
            throw new System.Exception();
        }

        public override bool Equals(object obj) { return Equals(obj as TN); } 
        public bool Equals(TN other) { return other != null && t.Equals(other.t); } 
        public override int GetHashCode() { return 831258139 + t.GetHashCode(); } 
        public static bool operator ==(TN left, TN right) { return EqualityComparer<TN>.Default.Equals(left, right); } 
        public static bool operator !=(TN left, TN right) { return !(left == right); }
    }
}
