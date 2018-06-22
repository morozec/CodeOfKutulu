using System;
using System.CodeDom;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Remoting.Messaging;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Win32.SafeHandles;

namespace CodeOfKutulu2
{
    class Player
    {
        #region A*
        public abstract class APoint : IComparable<APoint>
        {
            public double G { get; set; }
            public double H { get; set; }

            public double F
            {
                get
                {
                    return G + H;
                }
            }

            public APoint CameFromPoint { get; set; }

            public abstract IEnumerable<APoint> GetNeighbors(IEnumerable<APoint> points);

            public abstract double GetHeuristicCost(APoint goal);

            public abstract double GetCost(APoint goal);

            public int CompareTo(APoint other)
            {
                return -F.CompareTo(other.F);
            }

        }

        public class Point : APoint
        {
            public int X { get; set; }
            public int Y { get; set; }

            public int Weight { get; set; }

            public char Value { get; set; }

            public IEnumerable<APoint> Neighbours { get; set; }

            public override IEnumerable<APoint> GetNeighbors(IEnumerable<APoint> apoints)
            {
                return Neighbours;
                //var resPoints = new List<Point>();
                //foreach (var apoint in apoints)
                //{
                //    var point = (Point)apoint;
                //    if (point.Y == Y && (point.X == X - 1 || point.X == X + 1))
                //    {
                //        resPoints.Add(point);
                //    }
                //    else if (point.X == X && (point.Y == Y - 1 || point.Y == Y + 1))
                //    {
                //        resPoints.Add(point);
                //    }
                //}
                //return resPoints;
            }

            public override double GetHeuristicCost(APoint agoal)
            {
                var goal = (Point)agoal;
                return GetManhattenDist(X, Y, goal.X, goal.Y);
            }


            public override double GetCost(APoint agoal)
            {
                return (agoal as Point).Weight * GetHeuristicCost(agoal);
            }
        }

        public class Infrastructure
        {
            /// <summary>
            /// ����� ���������� ���
            /// </summary>
            public APoint Cps { get; private set; }
            /// <summary>
            /// ������ �������
            /// </summary>
            public IEnumerable<IList<APoint>> Channels { get; private set; }

            public Infrastructure(APoint cps, IEnumerable<IList<APoint>> channels)
            {
                Cps = cps;
                Channels = channels;
            }
        }

        /// <summary>
        /// ������� ��������������
        /// </summary>
        public class ExpansionMatrixConteiner
        {
            /// <summary>
            /// ��������� ������� �� ����� ����
            /// </summary>
            public IDictionary<APoint, double> ExpansionMatrix { get; set; }
            ///// <summary>
            ///// ����������� ���� ������� �� ����� ����
            ///// </summary>
            //public IDictionary<Point, IList<Point>> Path { get; set; } 
        }

        public static class Calculator
        {
            /// <summary>
            /// ������ ������� ���������������
            /// </summary>
            /// <param name="start">�����, ��� ������� �������������� ������� ���������������</param>
            /// <param name="goal">������� �����. ���� null, �� ������� ��������������� �������������� �� ��������� ����� �� ���� ��������� ����� ����</param>
            /// <param name="allPoints">��� ����� ����</param>
            /// <returns>������� ���������������</returns>
            private static ExpansionMatrixConteiner GetExpansionMatrix(APoint start, APoint goal, IEnumerable<APoint> allPoints)
            {
                foreach (var point in allPoints)
                {
                    point.CameFromPoint = null;
                }

                var emc = new ExpansionMatrixConteiner
                {
                    ExpansionMatrix = new Dictionary<APoint, double>(),
                    //Path =  new Dictionary<Point, IList<Point>>()
                };

                var closedSet = new HashSet<APoint>();
                var openSet = new HashSet<APoint> { start };

                start.G = 0d;
                start.H = goal == null ? 0d : start.GetHeuristicCost(goal);

                var pathFound = false;

                while (openSet.Count > 0)
                {
                    var x = GetPointWithMinF(openSet);
                    if (goal != null && x == goal)
                    {
                        pathFound = true;
                        break;
                    }
                    openSet.Remove(x);
                    closedSet.Add(x);
                    emc.ExpansionMatrix.Add(x, x.G);
                    //emc.Path.Add(x, ReconstructPath(x));

                    var neighbors = x.GetNeighbors(allPoints);
                    foreach (var y in neighbors)
                    {
                        if (closedSet.Contains(y)) continue;

                        var tentativeGScore = x.G + x.GetCost(y);
                        bool tentativeIsBetter;

                        if (!openSet.Contains(y))
                        {
                            openSet.Add(y);
                            tentativeIsBetter = true;
                        }
                        else
                        {
                            tentativeIsBetter = tentativeGScore < y.G;
                        }

                        if (tentativeIsBetter)
                        {
                            y.CameFromPoint = x;
                            y.G = tentativeGScore;
                            y.H = goal == null ? 0d : y.GetHeuristicCost(goal);
                        }
                    }
                }

                if (goal != null && !pathFound) throw new Exception("���� �� �������� ����� �� ������");


                return emc;
            }

            /// <summary>
            /// ������ ������������ ���� �� ������� �����
            /// </summary>
            /// <param name="start">��������� ����� ����</param>
            /// <param name="goal">������� ����� ����</param>
            /// <param name="allPoints">��� ����� ����</param>
            /// <returns>����������� ���� �� ��������� ����� �� �������</returns>
            public static IList<APoint> GetPath(APoint start, APoint goal, IEnumerable<APoint> allPoints)
            {
                GetExpansionMatrix(start, goal, allPoints);
                return ReconstructPath(goal);
            }

            /// <summary>
            /// ��������� ������ ��������������� ��� ������ ��������� �����
            /// </summary>
            /// <param name="startPoints">����� ��������� �����</param>
            /// <param name="allPoints">��� ����� ����</param>
            /// <returns>������� ��������������� ��� ��������� �����</returns>
            public static IDictionary<APoint, ExpansionMatrixConteiner> GetExpansionMatrices(IEnumerable<APoint> startPoints, IEnumerable<APoint> allPoints)
            {
                var result = new Dictionary<APoint, ExpansionMatrixConteiner>();
                foreach (var startPoint in startPoints)
                {
                    result.Add(startPoint, GetExpansionMatrix(startPoint, null, allPoints));
                }
                return result;
            }

            /// <summary>
            /// ��������� ����� � ����������� ��������� ����� ������������ ������ ���������������
            /// </summary>
            /// <param name="expansionMatrices">������� ���������������</param>
            /// <param name="allPoints">��� ����� ����</param>
            /// <returns>����� � ����������� ������</returns>
            private static APoint GetMinCostPoint(IDictionary<APoint, ExpansionMatrixConteiner> expansionMatrices, IEnumerable<APoint> allPoints)
            {

                var summCosts = new Dictionary<APoint, double>();
                foreach (var matrixPoint in allPoints)
                {
                    summCosts.Add(matrixPoint, 0d);
                    foreach (var startPoint in expansionMatrices.Keys)
                    {
                        summCosts[matrixPoint] += expansionMatrices[startPoint].ExpansionMatrix[matrixPoint];
                    }
                }

                APoint cps = null;
                var summCost = double.MaxValue;
                foreach (var matrixPoint in summCosts.Keys)
                {
                    if (summCosts[matrixPoint] < summCost)
                    {
                        cps = matrixPoint;
                        summCost = summCosts[matrixPoint];
                    }
                }

                return cps;
            }

            /// <summary>
            /// ��������� ����� � ����������� ��������� ������� �� (� ��) ������� 
            /// </summary>
            /// <param name="expansionMatrices">������� ���������������</param>
            /// <param name="notTraversedStartPoints">������ ������������ �����. ����� ��� ����� ����������� ����� �����������</param>
            /// <param name="collectionPoint">������� �����</param>
            /// <returns>����� � ����������� ��������� �������</returns>
            private static APoint GetNearestPoint(
                IDictionary<APoint, ExpansionMatrixConteiner> expansionMatrices,
                IEnumerable<APoint> notTraversedStartPoints,
                APoint collectionPoint)
            {
                APoint nearestPoint = null;
                var minCost = double.MaxValue;

                foreach (var point in notTraversedStartPoints)
                {
                    if (expansionMatrices[point].ExpansionMatrix[collectionPoint] < minCost)
                    {
                        nearestPoint = point;
                        minCost = expansionMatrices[point].ExpansionMatrix[collectionPoint];
                    }
                }

                return nearestPoint;
            }

            /// <summary>
            /// ���������� ���������� ������������
            /// </summary>
            /// <param name="startPoints">��������� (��������) �����</param>
            /// <param name="allPoints">��� ����� ����</param>
            /// <returns>����������� ��������� ������������</returns>
            public static Infrastructure GetInfrastructure(IEnumerable<APoint> startPoints, IEnumerable<APoint> allPoints)
            {
                var channels = new List<IList<APoint>>();

                //������ ������� ��������������� ��� ������
                var allExpansionMatrices = GetExpansionMatrices(startPoints, allPoints);
                //���������� ���������� ���
                var cps = GetMinCostPoint(allExpansionMatrices, allPoints);
                //������ ������� ��������������� ��� ���
                var cpsExpansionMatrixConteiner = GetExpansionMatrix(cps, null, allPoints);

                var notTraversedStartPoints = startPoints.ToList();

                //��������� ������� ��������������� ��� ��� � ������� ���� ����������� ������ ���������������
                if (!allExpansionMatrices.ContainsKey(cps))
                    allExpansionMatrices.Add(cps, cpsExpansionMatrixConteiner);
                //��� ���������� ������� �� ������ ���������� ������, ���� �� � ��� ����
                if (notTraversedStartPoints.Contains(cps))
                    notTraversedStartPoints.Remove(cps);

                //������� ������� (�� ���������) � ��� �����
                var nearestPoint = GetNearestPoint(allExpansionMatrices, notTraversedStartPoints, cps);

                if (nearestPoint == null)
                    return new Infrastructure(cps, new List<IList<APoint>>());

                //������ ����� �� ���������� ����� �� ���
                var path = ReconstructPath(cps, allExpansionMatrices[nearestPoint], allPoints);
                channels.Add(path);

                //������� ��������� ����� �� ������ ������������ �����
                notTraversedStartPoints.Remove(nearestPoint);

                while (notTraversedStartPoints.Any())
                {
                    //������� ������� (�� ���������) � ��� ����� ����� ������������
                    nearestPoint = GetNearestPoint(allExpansionMatrices, notTraversedStartPoints, cps);

                    //���� �����, ���������� ����������� (�� ���������) ������
                    IList<APoint> insetPointChannel = null;
                    var minInsetCost = double.MaxValue;
                    foreach (var channel in channels)
                    {
                        foreach (var point in channel)
                        {
                            var currentCost = allExpansionMatrices[nearestPoint].ExpansionMatrix[point];

                            if (currentCost < minInsetCost)
                            {
                                minInsetCost = currentCost;
                                insetPointChannel = channel;
                            }
                        }
                    }

                    if (insetPointChannel == null)
                        throw new Exception("����� ��� ������ �� ������");

                    //���� ����� �� �������� ��������� ������ ������, �������� ����� 3 ������� 
                    if (!nearestPoint.Equals(insetPointChannel.First()) && !nearestPoint.Equals(insetPointChannel.Last()))
                    {
                        //������� �������� ����� ������ ��� ���� ����� - ������ � ����� ������, ��������������� �� ������� ���� �����
                        var realInsetPoint = GetMinCostPoint(
                            new Dictionary<APoint, ExpansionMatrixConteiner>()
                            {
                            {insetPointChannel.First(), allExpansionMatrices[insetPointChannel.First()]},
                            {insetPointChannel.Last(), allExpansionMatrices[insetPointChannel.Last()]},
                            {nearestPoint, allExpansionMatrices[nearestPoint]},
                            },
                            allPoints
                            );

                        //��������� ������� ������ ���������������
                        if (!allExpansionMatrices.ContainsKey(realInsetPoint))
                        {
                            allExpansionMatrices.Add(realInsetPoint, GetExpansionMatrix(realInsetPoint, null, allPoints));
                        }

                        //�������� ������������ ����� �� 2 (�� ����� ������), � ����� ��������� ����� �� ���������������� ����� �� ����� ������
                        channels.Remove(insetPointChannel);
                        var firstChannelPointPath = ReconstructPath(
                            realInsetPoint,
                            allExpansionMatrices[insetPointChannel.First()],
                            allPoints);
                        var lastChannelPointPath = ReconstructPath(
                            realInsetPoint,
                            allExpansionMatrices[insetPointChannel.Last()],
                            allPoints);
                        var nearestPointPath = ReconstructPath(
                            realInsetPoint,
                            allExpansionMatrices[nearestPoint],
                            allPoints);
                        if (firstChannelPointPath.Count > 1)
                            channels.Add(firstChannelPointPath);
                        if (lastChannelPointPath.Count > 1)
                            channels.Add(lastChannelPointPath);
                        if (nearestPointPath.Count > 1)
                            channels.Add(nearestPointPath);
                    }
                    //������� �������������� ����� �� ������ ������������
                    notTraversedStartPoints.Remove(nearestPoint);
                }

                return new Infrastructure(cps, channels);

            }

            /// <summary>
            /// ����� ����� � ����������� ������������� �������� (F)
            /// </summary>
            /// <param name="points">������ �����</param>
            /// <returns>����� � ����������� ������������� ��������</returns>
            private static APoint GetPointWithMinF(IEnumerable<APoint> points)
            {
                if (!points.Any())
                {
                    throw new Exception("������ ������ �����");
                }
                var minF = double.MaxValue;
                APoint resultPoint = null;
                foreach (var point in points)
                {
                    if (point.F < minF)
                    {
                        minF = point.F;
                        resultPoint = point;
                    }
                }

                return resultPoint;
            }

            /// <summary>
            /// �������������� ������������ ����
            /// </summary>
            /// <param name="goal">������� �����</param>
            /// <returns>����������� ���� �� ������� �����</returns>
            private static IList<APoint> ReconstructPath(APoint goal)
            {
                var resultList = new List<APoint>();

                var currentPoint = goal;

                while (currentPoint != null)
                {
                    resultList.Add(currentPoint);
                    currentPoint = currentPoint.CameFromPoint;
                }

                resultList.Reverse();

                return resultList;
            }

            public static IList<APoint> ReconstructPath(APoint goal, ExpansionMatrixConteiner expansionMatrixConteiner, IEnumerable<APoint> allPoints)
            {
                var path = new List<APoint>() { goal };
                var currentPoint = goal;
                while (expansionMatrixConteiner.ExpansionMatrix[currentPoint] > 0)
                {
                    APoint closestNeighbour = null;
                    var minCost = double.MaxValue;
                    foreach (var neihgbour in currentPoint.GetNeighbors(allPoints))
                    {
                        if (expansionMatrixConteiner.ExpansionMatrix[neihgbour] < minCost)
                        {
                            minCost = expansionMatrixConteiner.ExpansionMatrix[neihgbour];
                            closestNeighbour = neihgbour;
                        }
                    }
                    currentPoint = closestNeighbour;
                    path.Add(closestNeighbour);
                }

                return path;
            }
        }

        #endregion

        #region units

        class CellPoint
        {
            public int X { get; set; }
            public int Y { get; set; }

            public CellPoint(int x, int y)
            {
                X = x;
                Y = y;
            }
        }

        abstract class Unit : CellPoint
        {
            public int Id { get; set; }

            protected Unit(int id, int x, int y) : base(x, y)
            {
                Id = id;
            }
        }

        class Explorer : Unit
        {
            public int Sanity { get; set; }
            public int Plans { get; set; }
            public int Lights { get; set; }

            public Explorer(int id, int x, int y, int sanity, int plans, int lights) : base(id, x, y)
            {
                Sanity = sanity;
                Plans = plans;
                Lights = lights;
            }
        }

        class Wanderer : Unit
        {
            public int Time { get; set; }
            public int State { get; set; }
            public int TargetId { get; set; }

            public Wanderer(int id, int x, int y, int time, int state, int targetId) : base(id, x, y)
            {
                Time = time;
                State = state;
                TargetId = targetId;
            }
        }

        class Shelter : Unit
        {
            public int RemainigEnergy { get; set; }

            public Shelter(int id, int x, int y, int remainigEnergy) : base(id, x, y)
            {
                RemainigEnergy = remainigEnergy;
            }
        }

        #endregion


        private class DamageItem
        {
            public Point Point { get; set; }
            public int Damage { get; set; }
            public IList<DamageItem> DamageChildren { get; set; }

            public int MinWandererDist { get; set; }
            public int MinWandererDistCount { get; set; }

            public int MinSlashersDist { get; set; }
            public int MinSlashersDistCount { get; set; }

            public int SumDamage => GetMinDamagePath().Sum(di => di.Damage);

            public IList<DamageItem> GetMinDamagePath()
            {
                var path = new List<DamageItem>(){this};
                if (!DamageChildren.Any()) return path;

                var minChildSumDamage = int.MaxValue;
                IList<DamageItem> minChildSumDamagePath = null;
                foreach (var child in DamageChildren)
                {
                    var childPath = child.GetMinDamagePath();
                    var childSumDamage = childPath.Sum(c => c.Damage);
                    if (childSumDamage < minChildSumDamage)
                    {
                        minChildSumDamage = childSumDamage;
                        minChildSumDamagePath = childPath;
                    }
                }
                path.AddRange(minChildSumDamagePath);
                return path;
            }
        }

        static int GetManhattenDist(int x1, int y1, int x2, int y2)
        {
            return Math.Abs(x1 - x2) + Math.Abs(y1 - y2);
        }

        static int GetManhattenDist(CellPoint p1, CellPoint p2)
        {
            return GetManhattenDist(p1.X, p1.Y, p2.X, p2.Y);
        }

        private static IList<Point> GetPointsCopy()
        {

            var res = new List<Point>();
            foreach (var point in Points)
            {
                var pointCopy = new Point() {X = point.X, Y = point.Y, Weight = point.Weight};
                res.Add(pointCopy);
            }

            foreach (var point in res)
            {
                point.Neighbours = res.Where(p =>
                    p.X == point.X && p.Y == point.Y - 1 ||
                    p.X == point.X && p.Y == point.Y + 1 ||
                    p.Y == point.Y && p.X == point.X - 1 ||
                    p.Y == point.Y && p.X == point.X + 1);
            }

            return res;
        }


        static void Main(string[] args)
        {
            string[] inputs;
            int width = int.Parse(Console.ReadLine());
            Console.Error.WriteLine(width);
            int height = int.Parse(Console.ReadLine());
            Console.Error.WriteLine(height);

            Points = new List<Point>();
            PointsTable = new Point[width, height];
            Point startPoint = null, finalPoint = null;

            for (int i = 0; i < height; i++)
            {
                string line = Console.ReadLine();
                Console.Error.WriteLine(line);
                for (int j = 0; j < line.Length; ++j)
                {
                    var ch = line[j];

                    var point = new Point()
                    {
                        X = j,
                        Y = i
                    };

                    var weight = 1;
                    if (ch == '#') weight = BigValue;
                    point.Weight = weight;

                    Points.Add(point);
                    PointsTable[j, i] = point;
                }
            }

            for (int i = 0; i < height; ++i)
            {
                for (int j = 0; j < width; ++j)
                {
                    var point = PointsTable[j, i];
                    var neighbours = new List<Point>();
                    if (i > 0) neighbours.Add(PointsTable[j,i-1]);
                    if (i < height - 1) neighbours.Add(PointsTable[j,i + 1]);
                    if (j > 0) neighbours.Add(PointsTable[j - 1, i]);
                    if (j < width - 1) neighbours.Add(PointsTable[j + 1, i]);
                    point.Neighbours = neighbours;
                }
            }

            var str = Console.ReadLine();
            Console.Error.WriteLine(str);
            inputs = str.Split(' ');

            int sanityLossLonely =
                int.Parse(inputs[0]); // how much sanity you lose every turn when alone, always 3 until wood 1
            int sanityLossGroup =
                int.Parse(inputs[
                    1]); // how much sanity you lose every turn when near another player, always 1 until wood 1
            int wandererSpawnTime =
                int.Parse(inputs[2]); // how many turns the wanderer take to spawn, always 3 until wood 1
            int wandererLifeTime =
                int.Parse(inputs[3]); // how many turns the wanderer is on map after spawning, always 40 until wood 1

            SanityLossLonely = sanityLossLonely;
            SanityLossGroup = sanityLossGroup;
            WandererSpawnTime = wandererSpawnTime;
            WandererLifeTime = wandererLifeTime;


            var ems = Calculator.GetExpansionMatrices(Points.Where(p => p.Weight < BigValue), Points.Where(p => p.Weight < BigValue));
            Pathes = new Dictionary<Point, IDictionary<Point, IList<APoint>>>();
            for (int i = 0; i < height; ++i)
            {
                for (int j = 0; j < width; ++j)
                {
                    var source = PointsTable[j, i];
                    if (source.Weight == BigValue) continue;
                    Pathes.Add(source, new Dictionary<Point, IList<APoint>>());
                    for (int k = 0; k < height; ++k)
                    {
                        for (int l = 0; l < width; ++l)
                        {
                            var dest = PointsTable[l, k];
                            if (dest.Weight == BigValue) continue;
                            var path = Calculator.ReconstructPath(dest, ems[source], Points);
                            Pathes[source].Add(dest, path);
                        }
                    }
                }
            }
            //var path = Calculator.ReconstructPath(Points[10], ems[Points[0]], Points);

                    // game loop
            while (true)
            {
                //var watch = System.Diagnostics.Stopwatch.StartNew();

                var points = GetPointsCopy();
                if (CurrentPlanCooldown > 0) CurrentPlanCooldown--;
                if (CurrentLightCooldown > 0) CurrentLightCooldown--;
                WandererPoints.Clear();
                ExplorerPoints.Clear();
                SlasherPoints.Clear();
                WandererExplorerPathDistances.Clear();

                Explorer myExplorer = null;
                IList<Explorer> allExplorers = new List<Explorer>();
                IList<Wanderer> wanderers = new List<Wanderer>();
                IList<Wanderer> slashers = new List<Wanderer>();
                IList<Shelter> shelters = new List<Shelter>();

                str = Console.ReadLine();
                Console.Error.WriteLine(str);
                int entityCount = int.Parse(str); // the first given entity corresponds to your explorer
                for (int i = 0; i < entityCount; i++)
                {
                    str = Console.ReadLine();
                    Console.Error.WriteLine(str);
                    inputs = str.Split(' ');
                    string entityType = inputs[0];
                    int id = int.Parse(inputs[1]);
                    int x = int.Parse(inputs[2]);
                    int y = int.Parse(inputs[3]);
                    int param0 = int.Parse(inputs[4]);
                    int param1 = int.Parse(inputs[5]);
                    int param2 = int.Parse(inputs[6]);

                    if (entityType == "EXPLORER")
                    {
                        var explorer = new Explorer(id, x, y, param0, param1, param2);
                        if (myExplorer == null)
                        {
                            myExplorer = explorer;
                            startPoint = points.Single(p => p.X == x && p.Y == y);
                        }

                        allExplorers.Add(explorer);
                        ExplorerPoints.Add(explorer.Id, PointsTable[explorer.X, explorer.Y]);
                    }
                    else if (entityType == "WANDERER")
                    {
                        var wanderer = new Wanderer(id, x, y, param0, param1, param2);
                        wanderers.Add(wanderer);
                        WandererPoints.Add(wanderer.Id, PointsTable[wanderer.X, wanderer.Y]);
                    }
                    else if (entityType == "SLASHER")
                    {
                        var slaher = new Wanderer(id, x, y, param0, param1, param2);
                        slashers.Add(slaher);
                        SlasherPoints.Add(slaher.Id, PointsTable[slaher.X, slaher.Y]);
                    }
                    else if (entityType == "EFFECT_SHELTER")
                    {
                        var shelter = new Shelter(id, x, y, param0);
                        shelters.Add(shelter);

                    }
                }

                if (allExplorers.Count == 1)
                {
                    Console.WriteLine("WAIT");
                    continue;
                }


                UpdateSlasherTargets(slashers, allExplorers);

                //var slasherDangerousPoints = GetSlasherDangerousPoints(allExplorers, slashers, points, myExplorer);
                //var wandererDangerousPoints = GetWandererDangerousPoints(myExplorer, wanderers, points);

                //foreach (var sdp in slasherDangerousPoints.Keys)
                //{
                //    sdp.Weight += SlasherWeight * slasherDangerousPoints[sdp];
                //}

                //foreach (var wdp in wandererDangerousPoints.Keys)
                //{
                //    wdp.Weight += WandererWeight * wandererDangerousPoints[wdp];
                //}

                //var goPoint = GetGoPoint(myExplorer, allExplorers, shelters, points);
                //finalPoint = points.Single(p => p.X == goPoint.X && p.Y == goPoint.Y);
                //var path = Calculator.GetPath(startPoint, finalPoint, points);

                //Console.Error.WriteLine($"{startPoint.X} {startPoint.Y} {startPoint.Weight}");
                //foreach (Point neighbour in startPoint.GetNeighbors(points))
                //    if (neighbour.Weight < BigValue)
                //        Console.Error.WriteLine($"{neighbour.X} {neighbour.Y} {neighbour.Weight}");
                
                
                var escapePoint = GetNewEscapePoint(myExplorer, allExplorers, wanderers, slashers, shelters.Where(s => s.RemainigEnergy > 0));
                if (escapePoint.X == myExplorer.X && escapePoint.Y == myExplorer.Y)
                {
                    var useYell = UseYell(myExplorer, allExplorers, wanderers, slashers);
                    var usePlan = UsePlan(myExplorer, allExplorers);
                    var useLight = UseLight(myExplorer, allExplorers, wanderers);
                    if (useYell)
                    {
                        IsYellUsed = true;
                        Console.WriteLine("YELL yell!!!");
                        continue;
                    }
                    if (usePlan)
                    {
                        CurrentPlanCooldown = PlanCooldown + 1;
                        Console.WriteLine("PLAN");
                        continue;
                    }

                    if (useLight)
                    {
                        CurrentLightCooldown = LightCooldown + 1;
                        Console.WriteLine("LIGHT");
                        continue;
                    }
                    Console.WriteLine("WAIT");
                }
                else
                {
                    Console.WriteLine($"MOVE {escapePoint.X} {escapePoint.Y}");
                }


                //var isBetterToHide = path.Count > 1 && escapePoint.Weight < (path[1] as Point).Weight;//опасно идти за эксплорером

                //var auraRange = goPoint is Shelter ? 0 : AuraRange;
                //var goPointDist = GetManhattenDist(myExplorer, goPoint);
                //if (goPointDist <= auraRange || isBetterToHide)
                //{
                //    var isCurrPointSafe = startPoint.Weight <= escapePoint.Weight;
                //    var shouldStay = escapePoint.X == myExplorer.X && escapePoint.Y == myExplorer.Y;
                    

                //    if (shouldStay || isCurrPointSafe)
                //    {
                //        var useYell = UseYell(myExplorer, allExplorers, wanderers, slashers);
                //        var usePlan = UsePlan(myExplorer, allExplorers);
                //        var useLight = UseLight(myExplorer, allExplorers, wanderers);
                //        if (useYell)
                //        {
                //            IsYellUsed = true;
                //            Console.WriteLine("YELL yell!!!");
                //            continue;
                //        }
                //        if (usePlan)
                //        {
                //            CurrentPlanCooldown = PlanCooldown + 1;
                //            Console.WriteLine("PLAN");
                //            continue;
                //        }

                //        if (useLight)
                //        {
                //            CurrentLightCooldown = LightCooldown + 1;
                //            Console.WriteLine("LIGHT");
                //            continue;
                //        }
                //    }
                    
                //    if (shouldStay)
                //    {
                //        //Console.Error.WriteLine("Curr point is most safety");
                //        Console.WriteLine("WAIT");
                //    }
                //    else
                //    {
                //        //Console.Error.WriteLine("Time to run");
                //        Console.WriteLine("MOVE " + escapePoint.X + " " + escapePoint.Y);
                //    }
                //}
                //else
                //{
                //    var step = path[1] as Point;
                //    Console.WriteLine("MOVE " + step.X + " " + step.Y);
                //}


                //watch.Stop();
                //var elapsedMs = watch.ElapsedMilliseconds;
            }
        }

        static int GetAuraRange(IDictionary<Unit, int> fds, int friendDistance)
        {
            var minDistUnits = fds.Keys.Where(x => fds[x] == friendDistance);
            if (minDistUnits.Any(u => u is Shelter)) return 0;
            return FriendAuraRange;
        }
       
        static Point GetNewEscapePoint(Explorer myExplorer, IList<Explorer> allExplorers,
            IList<Wanderer> wanderers, IList<Wanderer> slashers, IEnumerable<Shelter> activeShelters)
        {
            var damageItems = GetDamageItems(myExplorer, allExplorers, wanderers, slashers);
            DamageItem minDamageItem = damageItems[0];
            
            var minFds = new Dictionary<Unit, int>();
            var startPoint = PointsTable[myExplorer.X, myExplorer.Y];
            foreach (var e in allExplorers.Where(e => e.Id != myExplorer.Id))
            {
                var finalPoint = PointsTable[e.X, e.Y];
                var path = Pathes[startPoint][finalPoint]; 
                var dist = path.Count;
                minFds.Add(e, dist);
            }
            foreach (var s in activeShelters)
            {
                var finalPoint = PointsTable[s.X, s.Y];
                var path = Pathes[startPoint][finalPoint];
                var dist = path.Count;
                minFds.Add(s, dist);
            }
            var minFriendDistance = minFds.Values.Min();

            var tmpMin = minFds.Values.Where(d => d != minFriendDistance);
            var nextMinFd = tmpMin.Any() ? tmpMin.Min() : 0;

            //var minFriendDist = allExplorers.Where(e => e.Id != myExplorer.Id)
            //    .Min(e => GetManhattenDist(e.X, e.Y, damageItems[0].Point.X, damageItems[0].Point.Y));

            //var walkingWanderers = wanderers.Where(w => w.State == 1);
            //var minWandererDist = walkingWanderers.Any() ? walkingWanderers.Min(w =>
            //    GetManhattenDist(w.X, w.Y, minDamageItem.Point.X, minDamageItem.Point.Y)) : 0;
            //var minWandererCount = walkingWanderers.Any()
            //    ? walkingWanderers.Count(w =>
            //        GetManhattenDist(w.X, w.Y, minDamageItem.Point.X, minDamageItem.Point.Y) == minWandererDist) : 0;
            //var minDamage = int.MaxValue;
            //var minFriendDistance = int.MaxValue;

            for (int i = 1; i < damageItems.Count; ++i)
            {
                var di = damageItems[i];
                //var friendDist = allExplorers.Where(e => e.Id != myExplorer.Id)
                //    .Min(e => GetManhattenDist(e.X, e.Y, di.Point.X, di.Point.Y));
                var currStartPoint = PointsTable[di.Point.X, di.Point.Y];
                var fds = new Dictionary<Unit, int>();
                foreach (var e in allExplorers.Where(e => e.Id != myExplorer.Id))
                {
                    var finalPoint = PointsTable[e.X, e.Y];
                    var path = Pathes[currStartPoint][finalPoint];
                    var dist = path.Count;
                    fds.Add(e, dist);
                }

                foreach (var s in activeShelters)
                {
                    var finalPoint = PointsTable[s.X, s.Y];
                    var path = Pathes[startPoint][finalPoint];
                    var dist = path.Count;
                    fds.Add(s, dist);
                }

                var friendDistance = fds.Values.Min();
                var tmp = fds.Values.Where(d => d != friendDistance);
                var nextFd = tmp.Any() ? tmp.Min() : 0;

                //var wandererDist = walkingWanderers.Any() ? walkingWanderers.Min(w =>
                //    GetManhattenDist(w.X, w.Y, di.Point.X, di.Point.Y)) : 0;
                //var wandererCount = walkingWanderers.Any()
                //    ? walkingWanderers.Count(w =>
                //        GetManhattenDist(w.X, w.Y, di.Point.X, di.Point.Y) == wandererDist) : 0;


                //наименьший суммарный урон
                if (di.SumDamage < minDamageItem.SumDamage)
                {
                    minDamageItem = di;
                    minFds = fds;
                    minFriendDistance = friendDistance;
                    nextMinFd = nextFd;
                    continue;
                }
                if (di.SumDamage > minDamageItem.SumDamage) continue;

                //наиболее поздний урон
                var index = 0;
                var diPath = di.GetMinDamagePath();
                var minDiPath = minDamageItem.GetMinDamagePath();
                var isFinished = false;
                while (index < minDiPath.Count)
                {
                    if (diPath[index].Damage < minDiPath[index].Damage)
                    {
                        isFinished = true;
                        minDamageItem = di;
                        minFds = fds;
                        minFriendDistance = friendDistance;
                        nextMinFd = nextFd;
                        break;
                    }

                    if (diPath[index].Damage > minDiPath[index].Damage)
                    {
                        isFinished = true;
                        break;
                    }

                    index++;
                }
                if(isFinished) continue;

                //стоим в шелтерах
                index = 0;
                while (index < minDiPath.Count)
                {
                    var hasDiShelter =
                        activeShelters.Any(s => s.X == diPath[index].Point.X && s.Y == diPath[index].Point.Y);
                    var hasMinDiShelter =
                        activeShelters.Any(s => s.X == minDiPath[index].Point.X && s.Y == minDiPath[index].Point.Y);
                    
                    if (hasDiShelter && !hasMinDiShelter)
                    {
                        isFinished = true;
                        minDamageItem = di;
                        minFds = fds;
                        minFriendDistance = friendDistance;
                        nextMinFd = nextFd;
                        break;
                    }

                    if (!hasDiShelter && hasMinDiShelter)
                    {
                        isFinished = true;
                        break;
                    }

                    index++;
                }
                if (isFinished) continue;

                //ближе к другу, если мы вне ауры
                if (minFriendDistance > GetAuraRange(minFds, minFriendDistance))
                {
                    if (friendDistance < minFriendDistance)
                    {
                        minDamageItem = di;
                        minFds = fds;
                        minFriendDistance = friendDistance;
                        nextMinFd = nextFd;
                        continue;
                    }
                    if (friendDistance > minFriendDistance) continue;
                }
                //else if (friendDistance > GetAuraRange(fds, friendDistance)) continue;//не выходим из ауры

                //дальше от активных странников
                index = 0;
                while (index < minDiPath.Count)
                {
                    if (di.MinWandererDist > minDamageItem.MinWandererDist)
                    {
                        isFinished = true;
                        minDamageItem = di;
                        minFds = fds;
                        minFriendDistance = friendDistance;
                        nextMinFd = nextFd;
                        break;
                    }

                    if (di.MinWandererDist < minDamageItem.MinWandererDist)
                    {
                        isFinished = true;
                        break;
                    }

                    index++;
                }
                if (isFinished) continue;

                //меньше активных странников
                index = 0;
                while (index < minDiPath.Count)
                {
                    if (di.MinWandererDistCount > minDamageItem.MinWandererDistCount)
                    {
                        isFinished = true;
                        minDamageItem = di;
                        minFds = fds;
                        minFriendDistance = friendDistance;
                        nextMinFd = nextFd;
                        break;
                    }

                    if (di.MinWandererDistCount < minDamageItem.MinWandererDistCount)
                    {
                        isFinished = true;
                        break;
                    }

                    index++;
                }
                
                if (isFinished) continue;

                //ближе к другу, даже если в ауре
                if (friendDistance < minFriendDistance)
                {
                    minDamageItem = di;
                    minFds = fds;
                    minFriendDistance = friendDistance;
                    nextMinFd = nextFd;
                    continue;
                }
                if (friendDistance > minFriendDistance) continue;

                //дальше от слэшеров
                index = 0;
                while (index < minDiPath.Count)
                {
                    if (di.MinSlashersDist > minDamageItem.MinSlashersDist)
                    {
                        isFinished = true;
                        minDamageItem = di;
                        minFds = fds;
                        minFriendDistance = friendDistance;
                        nextMinFd = nextFd;
                        break;
                    }

                    if (di.MinSlashersDist < minDamageItem.MinSlashersDist)
                    {
                        isFinished = true;
                        break;
                    }

                    index++;
                }
                if (isFinished) continue;

                //меньше слэшеров
                index = 0;
                while (index < minDiPath.Count)
                {
                    if (di.MinSlashersDistCount > minDamageItem.MinSlashersDistCount)
                    {
                        isFinished = true;
                        minDamageItem = di;
                        minFds = fds;
                        minFriendDistance = friendDistance;
                        nextMinFd = nextFd;
                        break;
                    }

                    if (di.MinSlashersDistCount < minDamageItem.MinSlashersDistCount)
                    {
                        isFinished = true;
                        break;
                    }

                    index++;
                }
                if (isFinished) continue;

                //следующий ближайший друг
                if (nextFd < nextMinFd)
                {
                    minDamageItem = di;
                    minFds = fds;
                    minFriendDistance = friendDistance;
                    nextMinFd = nextFd;
                    continue;
                }
                if (nextFd > nextMinFd) continue;
            }
            

            return minDamageItem.Point;
        }
       

        static bool IsVisblePoint(CellPoint source, int destX, int destY)
        {
            if (source.X != destX && source.Y != destY) return false;
            if (source.X == destX)
            {
                var minY = Math.Min(source.Y, destY);
                var maxY = Math.Max(source.Y, destY);
                var currY = minY + 1;
                while (currY < maxY)
                {
                    var point = PointsTable[source.X, currY];
                    if (point.Weight == BigValue) return false;
                    currY++;
                }

                return true;
            }

            var minX = Math.Min(source.X, destX);
            var maxX = Math.Max(source.X, destX);
            var currX = minX + 1;
            while (currX < maxX)
            {
                var point = PointsTable[currX, source.Y];
                if (point.Weight == BigValue) return false;
                currX++;
            }

            return true;
        }

        static CellPoint GetSlasherTargetPoint(Wanderer slasher, IList<Explorer> allExplorers)
        {
            var targetExplorer = allExplorers.SingleOrDefault(e => e.Id == slasher.TargetId);

            if (targetExplorer != null && IsVisblePoint(slasher, targetExplorer.X, targetExplorer.Y))
                return targetExplorer;

            var visibleExplorers = allExplorers.Where(e => IsVisblePoint(slasher, e.X, e.Y)).ToList();
            var nearestVisibleExplorer = visibleExplorers.OrderBy(e => GetManhattenDist(slasher, e)).FirstOrDefault();
            if (nearestVisibleExplorer != null) return nearestVisibleExplorer;

            return null;//TODO: посторить путь до targetExplorer, если он есть
        }
        
        static bool UsePlan(
            Explorer myExplorer,
            IList<Explorer> allExplorers)
        {
            if (myExplorer.Plans == 0) return false;
            if (CurrentPlanCooldown > 0 || CurrentLightCooldown > 0) return false;

            //TODO: consider walls
            var closeExplorersCount = allExplorers.Count(
                e => e.Id != myExplorer.Id && GetManhattenDist(e, myExplorer) <= PlanDist);

            var restoringSanity = (PlanAddSanity + closeExplorersCount * PlanAddSanity) * PlanCooldown;
            var neededSanity = StartSanity - myExplorer.Sanity;//TODO: consider sanity loss
            if (restoringSanity > neededSanity) return false;

            if (closeExplorersCount < UsePlanExplorersCount && myExplorer.Sanity > CriticalPlanSanity)
                return false;

            return true;
        }

        static bool UseYell(Explorer myExplorer, IList<Explorer> allExplorers, IList<Wanderer> wanderers, IList<Wanderer> slashers)
        {
            if (IsYellUsed) return false;

            var profitCount = 0;
            foreach (var explorer in allExplorers.Where(e => e.Id != myExplorer.Id))
            {
                var myDist = GetManhattenDist(explorer, myExplorer);
                if (myDist > YellDist) continue;
                foreach (var wanderer in wanderers.Where(w => w.State > 0))
                {
                    var dist = GetManhattenDist(wanderer, explorer);
                    if (dist == 1)
                    {
                        profitCount++;
                    }
                    else if (dist == 2)
                    {
                        var target = allExplorers.SingleOrDefault(e => e.Id == wanderer.TargetId);
                        if (target != null && explorer.X == target.X && explorer.Y == target.Y)
                        {
                            profitCount++;
                        }
                    }
                }

                foreach (var slasher in slashers.Where(s => s.State == 3 || s.State == 2 && s.Time == 1))
                {
                    var slasherTargetPoint = GetSlasherTargetPoint(slasher, allExplorers);
                    if (slasherTargetPoint != null && slasherTargetPoint.X == explorer.X && slasherTargetPoint.Y == explorer.Y &&
                        slasherTargetPoint.X != myExplorer.X && slasherTargetPoint.Y != myExplorer.Y)
                        profitCount++;
                }
            }

            return profitCount >= YellExplorersCount;
        }

        //TODO: consider slashers
        static bool UseLight(
            Explorer myExplorer,
            IList<Explorer> allExplorers,
            IList<Wanderer> wanderers)
        {
            if (myExplorer.Lights == 0) return false;
            if (CurrentPlanCooldown > 0 || CurrentLightCooldown > 0) return false;

            IList<Wanderer> myExplorerTargetList = new List<Wanderer>();
            foreach (var wanderer in wanderers)
            {
                if (wanderer.State == 0) continue;
                var targetExplorer = allExplorers.SingleOrDefault(e => e.Id == wanderer.TargetId);

                if (targetExplorer != null && targetExplorer.X == myExplorer.X && targetExplorer.Y == myExplorer.Y)
                {
                    myExplorerTargetList.Add(wanderer);
                }
            }

            var noLightMyPoint = ExplorerPoints[myExplorer.Id];

            var points = GetPointsCopy();
            var myExplorerPoint = points.Single(p => p.X == myExplorer.X && p.Y == myExplorer.Y);
            var lightedPoints = GetLightedPoints(myExplorerPoint, points);
            foreach (var p in lightedPoints)
                p.Weight += LightAddDist;

            var canChangeTarget = false;

            foreach (var wanderer in myExplorerTargetList)
            {
                var startPoint = points.Single(p => p.X == wanderer.X && p.Y == wanderer.Y);
                var myPath = Calculator.GetPath(startPoint, myExplorerPoint, points);
                var myDist = myPath.Sum(p => (p as Point).Weight);
                foreach (var explorer in allExplorers)
                {
                    if (explorer.X == myExplorer.X && explorer.Y == myExplorer.Y) continue;

                    var finalPoint = points.Single(p => p.X == explorer.X && p.Y == explorer.Y);
                    var path = Calculator.GetPath(startPoint, finalPoint, points);

                    var dist = path.Sum(p => (p as Point).Weight);
                    if (myDist <= dist) continue;

                    if (path.Count <= 3) return true;

                    Point point2 = path[2] as Point;
                    var afterLightWandererPoint = PointsTable[point2.X, point2.Y];
                    var afterLightExplorerPoint = ExplorerPoints[explorer.Id];

                    var afterLightMyPath = Pathes[afterLightWandererPoint][noLightMyPoint];
                    var afterLightExplorerPath = Pathes[afterLightWandererPoint][afterLightExplorerPoint];

                    if (afterLightMyPath.Sum(p => (p as Point).Weight) >
                        afterLightExplorerPath.Sum(p => (p as Point).Weight))
                        return true;
                }
            }

            return canChangeTarget;
        }

        static IList<Point> GetLightedPoints(Point startPoint, IList<Point> points)
        {
            var res = new List<Point>() { startPoint };
            var currLine = new List<Point> { startPoint };
            for (var i = 0; i < LightRange; ++i)
            {
                var newLine = new List<Point>();
                foreach (var point in currLine)
                {
                    foreach (Point neighbour in point.GetNeighbors(points).Where(n => (n as Point).Weight == 1))
                    {
                        if (!res.Contains(neighbour))
                        {
                            res.Add(neighbour);
                            newLine.Add(neighbour);
                        }
                    }
                }

                currLine = newLine;
            }

            return res;
        }


        private static void UpdateSlasherTargets(IList<Wanderer> slashers, IList<Explorer> explorers)
        {
            foreach (var slasher in slashers)
            {
                var targetExplorer = explorers.SingleOrDefault(e => e.Id == slasher.TargetId);
                if (targetExplorer == null) continue;
                if (IsVisblePoint(slasher, targetExplorer.X, targetExplorer.Y))//м.б. невидим, если слешер только родился
                    LastTimeVisibleExplorers[slasher.Id] = targetExplorer; 
            }
        }

        private static IList<DamageItem> GetDamageItems(Explorer myExplorer, IList<Explorer> explorers, IList<Wanderer> wanderers, IList<Wanderer> slashers)
        {
            var damageItems = GetDamageItemsRec(myExplorer, explorers, wanderers, slashers, ExplorerPoints[myExplorer.Id], 0);
            foreach (var di in damageItems)
            {
                Console.Error.WriteLine($"{di.Point.X} {di.Point.Y} {di.SumDamage} {di.Damage} {di.MinWandererDist} {di.MinWandererDistCount} {di.MinSlashersDist} {di.MinSlashersDistCount}");
            }

            return damageItems;

        }

        private static IList<DamageItem> GetDamageItemsRec(Explorer myExplorer, IList<Explorer> explorers,
            IList<Wanderer> wanderers, IList<Wanderer> slashers, Point sourcePoint, int depth)
        {
            var damageItems = new List<DamageItem>();

            var currentPossibleMoves = GetCurrentPossibleMoves(sourcePoint);
            foreach (var move in currentPossibleMoves)
            {
                var myCurrExplorer = new Explorer(myExplorer.Id,
                    move.X,
                    move.Y,
                    myExplorer.Sanity,
                    myExplorer.Plans,
                    myExplorer.Lights);
                var newExplorers = GetNewExplorers(explorers, wanderers, slashers, myCurrExplorer);
                var newWanderes = GetNewWanderes(newExplorers, wanderers);
                var newSlashers = GetNewSlashers(newExplorers, slashers);

                var damage = newWanderes.Count(w => w.X == move.X && w.Y == move.Y);
                damage += newSlashers.Count(s => s.State == 4 && s.Time == 6 && s.X == move.X && s.Y == move.Y);

                var minWandererDist = newWanderes.Any() ? newWanderes.Min(w => GetManhattenDist(w.X, w.Y, move.X, move.Y)) : 0;
                var minWandererDistCount = newWanderes.Any() ? newWanderes.Count(w => GetManhattenDist(w.X, w.Y, move.X, move.Y) == minWandererDist) : 0;

                var minSlahersDist = newSlashers.Any() ? newSlashers.Min(w => GetManhattenDist(w.X, w.Y, move.X, move.Y)) : 0;
                var minSlahersDistCount = newSlashers.Any() ? newSlashers.Count(w => GetManhattenDist(w.X, w.Y, move.X, move.Y) == minSlahersDist) : 0;

                var damageItem = new DamageItem()
                {
                    Damage = damage,
                    Point = move,
                    MinWandererDist = minWandererDist,
                    MinWandererDistCount = minWandererDistCount,
                    MinSlashersDist = minSlahersDist,
                    MinSlashersDistCount = minSlahersDistCount
                };

                if (depth == PredictionDepth)
                {
                    damageItem.DamageChildren = new List<DamageItem>();
                    continue;
                }


                newWanderes = newWanderes.Where(w => !newExplorers.Any(s => GetManhattenDist(w, s) == 0)).ToList();//удаляем странников, дошедших до цели
                var nextDamages = GetDamageItemsRec(myExplorer, newExplorers, newWanderes, newSlashers, move, depth + 1);
                damageItem.DamageChildren = nextDamages;

                damageItems.Add(damageItem);
            }

            return damageItems;

        }

        private static IList<Point> GetCurrentPossibleMoves(Point sourcePoint)
        {
            var possibleMoves = new List<Point>() { sourcePoint };
            foreach (Point n in sourcePoint.GetNeighbors(Points))
            {
                if (n.Weight == BigValue) continue;
                possibleMoves.Add(n);
            }

            return possibleMoves;
        }

        private static IList<Explorer> GetNewExplorers(IList<Explorer> explorers, IList<Wanderer> wanderers, IList<Wanderer> slashers, Explorer myExplorer)
        {
            var newExplorers = new List<Explorer>();
             
            foreach (var explorer in explorers)
            {
                if (explorer.Id == myExplorer.Id)
                {
                    newExplorers.Add(myExplorer);
                    continue;
                }

                var explorerPoint = PointsTable[explorer.X, explorer.Y]; 
                var neighbours = explorerPoint.GetNeighbors(Points);
                var minDamagePoint = explorerPoint;
                var minDamage = wanderers.Count(w => GetManhattenDist(w, explorer) <= 1);
                var walkingWanderers = wanderers.Where(w => w.State == 1);
                var minWanderersDist = walkingWanderers.Any()
                    ? walkingWanderers.Min(w => GetManhattenDist(w.X, w.Y, minDamagePoint.X, minDamagePoint.Y)) : 0;
                var minVisibleSlashers = slashers.Count(s => IsVisblePoint(s, minDamagePoint.X, minDamagePoint.Y));

                foreach (var slasher in slashers.Where(s => s.State == 3)) //Не проверяем TargetId - считаем, что целевой эксплорер может уйти
                {
                    if (IsVisblePoint(slasher, explorerPoint.X, explorerPoint.Y)) minDamage++;
                }

                foreach (Point n in neighbours)
                {
                    if (n.Weight == BigValue) continue;
                    var damage = wanderers.Count(w => GetManhattenDist(w.X, w.Y, n.X, n.Y) <= 1);
                    var wanderersDist = walkingWanderers.Any()
                        ? walkingWanderers.Min(w => GetManhattenDist(w.X, w.Y, n.X, n.Y)) : 0;
                    var visibleSlashers = slashers.Count(s => IsVisblePoint(s, n.X, n.Y));

                    foreach (var slasher in slashers.Where(s => s.State == 3))
                    {
                        if (IsVisblePoint(slasher, n.X, n.Y)) damage++;
                    }
                        
                    if (wanderersDist > minWanderersDist)
                    {
                        minDamage = damage;
                        minDamagePoint = n;
                        minWanderersDist = wanderersDist;
                        minVisibleSlashers = visibleSlashers;
                        continue;
                    }
                    if (wanderersDist < minWanderersDist) continue;

                    if (damage < minDamage)
                    {
                        minDamage = damage;
                        minDamagePoint = n;
                        minWanderersDist = wanderersDist;
                        minVisibleSlashers = visibleSlashers;
                        continue;
                    }
                    if (damage > minDamage) continue;

                    if (visibleSlashers < minVisibleSlashers)
                    {
                        minDamage = damage;
                        minDamagePoint = n;
                        minWanderersDist = wanderersDist;
                        minVisibleSlashers = visibleSlashers;
                        continue;
                    }
                    if (visibleSlashers > minVisibleSlashers) continue;


                }
                var newExplorer = new Explorer(explorer.Id, minDamagePoint.X, minDamagePoint.Y, explorer.Sanity, explorer.Plans, explorer.Lights);
                newExplorers.Add(newExplorer);
            }

            return newExplorers;
        }


        private static IList<Wanderer> GetNewWanderes(IList<Explorer> explorers, IList<Wanderer> wanderers)
        {
            var newWanderes = new List<Wanderer>();
            foreach (var wanderer in wanderers.Where(w => w.State == 1)) 
            {
                if (wanderer.Time == 0) continue;

                var pathCounts = new Dictionary<Explorer, int>();
                foreach (var e in explorers)
                {
                    var path = Pathes[PointsTable[wanderer.X, wanderer.Y]][PointsTable[e.X, e.Y]]; 
                    pathCounts.Add(e, path.Count);
                }

                var minPathCount = pathCounts.Values.Min();
                var nearestExplorers = explorers.Where(e => pathCounts[e] == minPathCount).ToList(); 

                Explorer targetExplorer = nearestExplorers.SingleOrDefault(e => e.Id == wanderer.TargetId);
                if (targetExplorer == null) targetExplorer = nearestExplorers.First();//TODO

                Point nearestPoint = PointsTable[wanderer.X, wanderer.Y];

                var startPoint = PointsTable[wanderer.X, wanderer.Y];
                var finalPoint = PointsTable[targetExplorer.X, targetExplorer.Y];
                int minDist = Pathes[startPoint][finalPoint].Count; 
                //if (manhDist > ManhDistToUseAStar)
                //{
                //    minDist = GetManhattenDist(nearestPoint.X,
                //        nearestPoint.Y,
                //        targetExplorer.X,
                //        targetExplorer.Y);
                //}
                //else
                //{
                    
                //    var path = Calculator.GetPath(startPoint, finalPoint, Points);
                //    minDist = path.Count;
                //}

                if (nearestPoint.X != targetExplorer.X || nearestPoint.Y != targetExplorer.Y)
                {
                    foreach (Point n in nearestPoint.Neighbours)
                    {
                        if (n.Weight == BigValue) continue;
                        //if (manhDist > ManhDistToUseAStar)
                        //{
                        //    dist = GetManhattenDist(n.X, n.Y, targetExplorer.X, targetExplorer.Y);
                        //}
                        //else
                        //{
                        //    var startPoint = PointsTable[n.X, n.Y];
                        //    var path = Calculator.GetPath(startPoint, finalPoint, Points);
                        //    dist = path.Count;
                        //}
                        var nStartPoint = PointsTable[n.X, n.Y];
                        var dist = Pathes[nStartPoint][finalPoint].Count;

                        if (dist < minDist)
                        {
                            minDist = dist;
                            nearestPoint = n;
                        }

                    }
                }

                var newWanderer = new Wanderer(wanderer.Id, nearestPoint.X, nearestPoint.Y, wanderer.Time - 1, wanderer.State, wanderer.TargetId);
                newWanderes.Add(newWanderer);
            }

            return newWanderes;
        }

        private static IList<Wanderer> GetNewSlashers(IList<Explorer> explorers, IList<Wanderer> slashers)
        {
            var newSlashers = new List<Wanderer>();
            foreach (var slasher in slashers)
            {
                var nearestVisibleExplorer = explorers.Where(e => IsVisblePoint(slasher, e.X, e.Y))
                    .OrderBy(e => GetManhattenDist(slasher, e)).FirstOrDefault(); //TODO

                if (slasher.State == 3) //прыгает
                {
                    int newSlasherX, newSlasherY;
                    var targetExplorer = explorers.SingleOrDefault(e => e.Id == slasher.TargetId);
                    if (targetExplorer != null)
                    {
                        if (IsVisblePoint(slasher, targetExplorer.X, targetExplorer.Y))
                        {
                            newSlasherX = targetExplorer.X;
                            newSlasherY = targetExplorer.Y;
                        }
                        else
                        {
                            if (nearestVisibleExplorer != null)
                            {
                                newSlasherX = nearestVisibleExplorer.X;
                                newSlasherY = nearestVisibleExplorer.Y;
                            }
                            else
                            {
                                if (LastTimeVisibleExplorers.ContainsKey(slasher.Id))
                                {
                                    newSlasherX = LastTimeVisibleExplorers[slasher.Id].X;
                                    newSlasherY = LastTimeVisibleExplorers[slasher.Id].Y;
                                }
                                else
                                {
                                    newSlasherX = slasher.X;
                                    newSlasherY = slasher.Y;
                                }
                            }
                        }
                    }
                    else//никого не видно перед прыжком. но кто-то мог выйти на LoS
                    {
                        if (nearestVisibleExplorer != null)
                        {
                            newSlasherX = nearestVisibleExplorer.X;
                            newSlasherY = nearestVisibleExplorer.Y;
                        }
                        else
                        {
                            if (LastTimeVisibleExplorers.ContainsKey(slasher.Id))
                            {
                                newSlasherX = LastTimeVisibleExplorers[slasher.Id].X;
                                newSlasherY = LastTimeVisibleExplorers[slasher.Id].Y;
                            }
                            else
                            {
                                newSlasherX = slasher.X;
                                newSlasherY = slasher.Y;
                            }
                        }
                    }
                    

                    var newSlasher = new Wanderer(slasher.Id, newSlasherX, newSlasherY, 6, 4, -1);
                    newSlashers.Add(newSlasher);
                }
                else if (slasher.State == 2) //готовится к прыжку
                {
                    var newTime = slasher.Time - 1;
                    var newState = newTime == 0 ? 3 : 2;
                    int newTargetExplorerId = -1;

                    var targetExplorer = explorers.SingleOrDefault(e => e.Id == slasher.TargetId);
                    if (targetExplorer != null)
                    {
                        if (IsVisblePoint(slasher, targetExplorer.X, targetExplorer.Y))
                        {
                            newTargetExplorerId = targetExplorer.Id;
                        }
                        else if (nearestVisibleExplorer != null)
                        {
                            newTargetExplorerId = nearestVisibleExplorer.Id;
                        }
                    }
                    
                    var newSlasher = new Wanderer(slasher.Id, slasher.X, slasher.Y, newTime, newState, newTargetExplorerId);
                    newSlashers.Add(newSlasher);

                }
                else if (slasher.State == 0) //зарождается
                {
                    var newTime = slasher.Time - 1;
                    int newState;
                    int newTargetExplorerId;

                    if (newTime > 0)
                    {
                        newState = 0;
                        newTargetExplorerId = -1;
                    }
                    else
                    {
                        if (nearestVisibleExplorer != null)
                        {
                            newState = 3;
                            newTargetExplorerId = nearestVisibleExplorer.Id;
                        }
                        else
                        {
                            newState = 1;
                            newTargetExplorerId = -1;
                        }
                    }
                    var newSlasher = new Wanderer(slasher.Id, slasher.X, slasher.Y, newTime, newState, newTargetExplorerId);
                    newSlashers.Add(newSlasher);
                }
                //else if (slasher.State == 1)
                //{
                //    var explorerDists = new Dictionary<Explorer, int>();
                //    var startPoint = PointsTable[slasher.X, slasher.Y];
                //    foreach (var e in explorers)
                //    {
                //        var finalPoint = PointsTable[e.X, e.Y];
                //        var path = Pathes[startPoint][finalPoint];
                //        var dist = path.Count;
                //        explorerDists.Add(e, dist);
                //    }

                //    var minDist = explorerDists.Values.Min();
                //    var nearestExplorers = explorerDists.Keys.Where(e => explorerDists[e] == minDist);
                //    var visibleExplorers = nearestExplorers.Where(e => IsVisblePoint(slasher, e.X, e.Y));

                //    var targetExplorer = visibleExplorers.FirstOrDefault();
                //    if (targetExplorer != null)
                //    {
                //        var newSlasher = new Wanderer(slasher.Id, slasher.X, slasher.Y, 2, 2, targetExplorer.Id);
                //        newSlashers.Add(newSlasher);
                //    }
                //    else
                //    {
                //        var nearestExplorer = nearestExplorers.First();
                //        var finalPoint = PointsTable[nearestExplorer.X, nearestExplorer.Y];
                //        var path = Pathes[startPoint][finalPoint];

                //        var x = path.Count == 1 ? nearestExplorer.X : (path[1] as Point).X;
                //        var y = path.Count == 1 ? nearestExplorer.Y : (path[1] as Point).Y;

                //        var newSlasher = new Wanderer(slasher.Id, x, y, slasher.Time, 1, nearestExplorer.Id);
                //        newSlashers.Add(newSlasher);
                //    }

                //}
            }

            return newSlashers;

        }



        #region static and constants values

        private static IList<Point> Points { get; set; }
        private static Point[,] PointsTable { get; set; }
        private static IDictionary<int, Point> ExplorerPoints = new Dictionary<int, Point>();
        private static IDictionary<int, Point> WandererPoints = new Dictionary<int, Point>();
        private static IDictionary<int, Point> SlasherPoints = new Dictionary<int, Point>();

        private static IDictionary<int, Dictionary<int, int>> WandererExplorerPathDistances = new Dictionary<int, Dictionary<int, int>>();
        private static IDictionary<int, Explorer> LastTimeVisibleExplorers = new Dictionary<int, Explorer>();

        private const int BigValue = 999999;
        private const int SlasherWeight = 1000;
        private const int WandererWeight = 1000;

        private static int CurrentPlanCooldown = 0;
        private static int CurrentLightCooldown = 0;
        private static bool IsYellUsed = false;

        private static int SanityLossLonely = -1;
        private static int SanityLossGroup = -1;
        private static int WandererSpawnTime = -1;
        private static int WandererLifeTime = -1;

        private const int StartSanity = 250;
        private const int PlanDist = 2;
        private const int PlanCooldown = 5;
        private const int PlanAddSanity = 3;

        private const int UsePlanExplorersCount = 2;
        private const int CriticalPlanSanity = 100;

        private const int YellDist = 1;
        private const int YellStanTime = 2;

        private const int YellExplorersCount = 3;//TODO: up!

        private const int LightAddDist = 4;
        private const int LightRange = 5;
        private const int LightCooldown = 3;

        private const int FriendAuraRange = 2;

        private const int ShelterDefaultEnergy = 10;
        private const int MaxShelterEnergyLeak = 3;

        private const int ManhDistToUseAStar = 4;
        private const int PredictionDepth = 4;

        //private static IDictionary<APoint, ExpansionMatrixConteiner> Ems;
        private static IDictionary<Point, IDictionary<Point, IList<APoint>>> Pathes;

        #endregion

    }
}
