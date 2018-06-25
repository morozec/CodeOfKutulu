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

            public int YellTime { get; set; }

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
            public int MyDamage { get; set; }

            private IList<DamageItem> _damageChildren;

            public IList<DamageItem> DamageChildren
            {
                get { return _damageChildren; }
                set
                {
                    _damageChildren = value;
                    MinDamagePath = GetMinDamagePath();
                }
            }

            public int MinWandererDist { get; set; }
            public int MinWandererDistCount { get; set; }

            public bool CanBeYelled { get; set; }

            public int NewWanderersCount { get; set; }

            public int YelledExplorersDamage { get; set; }

            public int SumDamage => MinDamagePath.Sum(di => di.Damage);
            public int MySumDamage => MinDamagePath.Sum(di => di.MyDamage);
            public int YelledEplorersSumDamage => MinDamagePath.Sum(di => di.YelledExplorersDamage);

            public List<DamageItem> MinDamagePath { get; private set; }

            private List<DamageItem> GetMinDamagePath()
            {
                var path = new List<DamageItem>() { this };
                if (!DamageChildren.Any()) return path;


                List<DamageItem> minChildSumDamagePath = null;
                //                if (!isYelled && CanBeYelled)
                //                {
                //                    minChildSumDamagePath = new List<DamageItem>(){DamageChildren[0]};
                //                    if (DamageChildren[0].DamageChildren.Any())
                //                    {
                //                        minChildSumDamagePath.AddRange(DamageChildren[0].DamageChildren[0].GetMinDamagePath(true));
                //                    }
                //                }
                //                else
                //                {
                var minChildSumDamage = int.MaxValue;
                foreach (var child in DamageChildren)
                {
                    var childPath = child.GetMinDamagePath();
                    var childSumDamage = childPath.Sum(c => c.MyDamage);
                    if (childSumDamage < minChildSumDamage)
                    {
                        minChildSumDamage = childSumDamage;
                        minChildSumDamagePath = childPath;
                    }
                }
                //}

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
                    if (i > 0) neighbours.Add(PointsTable[j, i - 1]);//up
                    if (j < width - 1) neighbours.Add(PointsTable[j + 1, i]);//right
                    if (i < height - 1) neighbours.Add(PointsTable[j, i + 1]);//down
                    if (j > 0) neighbours.Add(PointsTable[j - 1, i]);//left

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
            Pathes = new IList<APoint>[width, height][,];
            //Pathes = new Dictionary<Point, IDictionary<Point, IList<APoint>>>();
            for (int i = 0; i < height; ++i)
            {
                for (int j = 0; j < width; ++j)
                {
                    var source = PointsTable[j, i];
                    if (source.Weight == BigValue) continue;
                    Pathes[j, i] = new IList<APoint>[width, height];
                    for (int k = 0; k < height; ++k)
                    {
                        for (int l = 0; l < width; ++l)
                        {
                            var dest = PointsTable[l, k];
                            if (dest.Weight == BigValue) continue;
                            var path = Calculator.ReconstructPath(source, ems[dest], Points);
                            Pathes[j, i][l, k] = path;
                        }
                    }
                }
            }

            //            LightedLengths = new int[width,height][,];
            //            for (int i = 0; i < height; ++i)
            //            {
            //                for (int j = 0; j < width; ++j)
            //                {
            //                    var source = PointsTable[j, i];
            //                    if (source.Weight == BigValue) continue;
            //                    LightedLengths[j,i] = new int[width,height];
            //                    var lightedPoints = GetLightedPoints(source, Points);
            //                    foreach (var p in lightedPoints)
            //                        p.Weight += LightAddDist;
            //                    for (int k = 0; k < height; ++k)
            //                    {
            //                        for (int l = 0; l < width; ++l)
            //                        {
            //                            var dest = PointsTable[l, k];
            //                            if (dest.Weight == BigValue) continue;
            //                            var path = Calculator.GetPath(dest, source, Points);
            //                            LightedLengths[j, i][l, k] = path.Count;
            //                        }
            //                    }
            //                    foreach (var p in lightedPoints)
            //                        p.Weight = 1;
            //                }
            //            }

            // game loop
            while (true)
            {
                //var watch = System.Diagnostics.Stopwatch.StartNew();

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
                var amIYelled = false;
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
                    else if (entityType == "EFFECT_YELL")
                    {
                        if (param2 == myExplorer.Id)
                        {
                            YelledMeIds.Add(param1);
                            amIYelled = true;
                        }
                        var targetExplorer = allExplorers.SingleOrDefault(e => e.Id == param2);
                        if (targetExplorer != null) targetExplorer.YellTime = 1;
                    }
                }

                if (allExplorers.Count == 1)
                {
                    Console.WriteLine("WAIT");
                    continue;
                }


                UpdateSlasherTargets(slashers, allExplorers);

                if (amIYelled)
                {
                    Console.WriteLine("WAIT I am yelled");
                    continue;
                }

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

                var useYell = NewUseYell(myExplorer, allExplorers, wanderers, slashers);
                if (useYell)
                {
                    Console.WriteLine("YELL qwerty");
                    continue;
                }

                var escapePoint = GetNewEscapePoint(myExplorer, allExplorers, wanderers, slashers, shelters, out var waitDamageItem);

                if (waitDamageItem.MySumDamage == 0)
                {
                    var canBeYelled = CanBeYelled(myExplorer, allExplorers);
                    if (UsePlan(myExplorer, allExplorers, wanderers, canBeYelled))
                    {
                        CurrentPlanCooldown = PlanCooldown + 1;
                        Console.WriteLine("PLAN");
                        continue;
                    }

                    if (NewUseLight(myExplorer, allExplorers, wanderers, canBeYelled))
                    {
                        CurrentLightCooldown = LightCooldown + 1;
                        Console.WriteLine("LIGHT");
                        continue;
                    }
                }
                if (escapePoint.X == myExplorer.X && escapePoint.Y == myExplorer.Y)
                {
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
                //Console.Error.WriteLine(elapsedMs);
            }
        }

        static int GetAuraRange(IDictionary<Unit, int> fds, int friendDistance)
        {
            var minDistUnits = fds.Keys.Where(x => fds[x] == friendDistance);
            if (minDistUnits.Any(u => u is Shelter)) return 0;
            return FriendAuraRange;
        }

        static Point GetNewEscapePoint(Explorer myExplorer, IList<Explorer> allExplorers,
            IList<Wanderer> wanderers, IList<Wanderer> slashers, IEnumerable<Shelter> shelters, out DamageItem waitDamageItem)
        {
            var damageItems = GetDamageItems(myExplorer, allExplorers, wanderers, slashers);
            DamageItem minDamageItem = damageItems[0];
            waitDamageItem = damageItems[0];
            var needSearchExplorer = allExplorers.Count > 2 || !shelters.Any();
            var seearchableShelters = shelters.Where(s => s.RemainigEnergy > 0);
            if (!seearchableShelters.Any()) seearchableShelters = shelters;

            var minFds = new Dictionary<Unit, int>();
            if (needSearchExplorer)
                foreach (var e in allExplorers.Where(e => e.Id != myExplorer.Id))
                {
                    var path = Pathes[myExplorer.X, myExplorer.Y][e.X, e.Y];
                    var dist = path.Count - 1;
                    minFds.Add(e, dist);
                }
            foreach (var s in seearchableShelters)
            {
                var path = Pathes[myExplorer.X, myExplorer.Y][s.X, s.Y];
                var dist = path.Count - 1;
                minFds.Add(s, dist);
            }
            var minFriendDistance = minFds.Values.Min();

            var tmpMin = minFds.Values.Where(d => d != minFriendDistance);
            var nextMinFd = tmpMin.Any() ? tmpMin.Min() : 0;

            var minOrderedWanderers = wanderers.OrderBy(w => Pathes[w.X, w.Y][myExplorer.X, myExplorer.Y].Count).ToList();

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
                var fds = new Dictionary<Unit, int>();
                if (needSearchExplorer)
                    foreach (var e in allExplorers.Where(e => e.Id != myExplorer.Id))
                    {
                        var path = Pathes[di.Point.X, di.Point.Y][e.X, e.Y];
                        var dist = path.Count - 1;
                        fds.Add(e, dist);
                    }

                foreach (var s in seearchableShelters)
                {
                    var path = Pathes[di.Point.X, di.Point.Y][s.X, s.Y];
                    var dist = path.Count - 1;
                    fds.Add(s, dist);
                }

                var friendDistance = fds.Values.Min();
                var tmp = fds.Values.Where(d => d != friendDistance);
                var nextFd = tmp.Any() ? tmp.Min() : 0;
                var orderedWanderers = wanderers.OrderBy(w => Pathes[w.X, w.Y][di.Point.X, di.Point.Y].Count).ToList();
                //var wandererDist = walkingWanderers.Any() ? walkingWanderers.Min(w =>
                //    GetManhattenDist(w.X, w.Y, di.Point.X, di.Point.Y)) : 0;
                //var wandererCount = walkingWanderers.Any()
                //    ? walkingWanderers.Count(w =>
                //        GetManhattenDist(w.X, w.Y, di.Point.X, di.Point.Y) == wandererDist) : 0;


                //наименьший суммарный урон
                if (di.MySumDamage < minDamageItem.MySumDamage)
                {
                    minDamageItem = di;
                    minFds = fds;
                    minFriendDistance = friendDistance;
                    nextMinFd = nextFd;
                    minOrderedWanderers = orderedWanderers;
                    continue;
                }
                if (di.MySumDamage > minDamageItem.MySumDamage) continue;

                //наиболее поздний урон
                var index = 0;
                var diPath = di.MinDamagePath;
                var minDiPath = minDamageItem.MinDamagePath;
                var isFinished = false;
                while (index < minDiPath.Count)
                {
                    if (diPath[index].MyDamage < minDiPath[index].MyDamage)
                    {
                        isFinished = true;
                        minDamageItem = di;
                        minFds = fds;
                        minFriendDistance = friendDistance;
                        nextMinFd = nextFd;
                        minOrderedWanderers = orderedWanderers;
                        break;
                    }

                    if (diPath[index].MyDamage > minDiPath[index].MyDamage)
                    {
                        isFinished = true;
                        break;
                    }

                    index++;
                }
                if (isFinished) continue;



                //стоим в шелтерах
                index = 0;
                while (index < minDiPath.Count)
                {
                    var hasDiShelter =
                        seearchableShelters.Any(s => s.X == diPath[index].Point.X && s.Y == diPath[index].Point.Y);
                    var hasMinDiShelter =
                        seearchableShelters.Any(s => s.X == minDiPath[index].Point.X && s.Y == minDiPath[index].Point.Y);

                    if (hasDiShelter && !hasMinDiShelter)
                    {
                        isFinished = true;
                        minDamageItem = di;
                        minFds = fds;
                        minFriendDistance = friendDistance;
                        nextMinFd = nextFd;
                        minOrderedWanderers = orderedWanderers;
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
                var minAuraRange = GetAuraRange(minFds, minFriendDistance);
                var auraRange = GetAuraRange(fds, friendDistance);
                if (friendDistance < minFriendDistance && minFriendDistance > minAuraRange)
                {
                    minDamageItem = di;
                    minFds = fds;
                    minFriendDistance = friendDistance;
                    nextMinFd = nextFd;
                    minOrderedWanderers = orderedWanderers;
                    continue;
                }
                if (friendDistance > minFriendDistance && friendDistance > auraRange)
                    continue;

                //if (minFriendDistance > GetAuraRange(minFds, minFriendDistance))
                //{
                //    if (friendDistance < minFriendDistance)
                //    {
                //        minDamageItem = di;
                //        minFds = fds;
                //        minFriendDistance = friendDistance;
                //        nextMinFd = nextFd;
                //        continue;
                //    }
                //    if (friendDistance > minFriendDistance) continue;
                //}
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
                        minOrderedWanderers = orderedWanderers;
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
                    if (di.MinWandererDistCount < minDamageItem.MinWandererDistCount)
                    {
                        isFinished = true;
                        minDamageItem = di;
                        minFds = fds;
                        minFriendDistance = friendDistance;
                        nextMinFd = nextFd;
                        minOrderedWanderers = orderedWanderers;
                        break;
                    }

                    if (di.MinWandererDistCount > minDamageItem.MinWandererDistCount)
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
                    minOrderedWanderers = orderedWanderers;
                    continue;
                }
                if (friendDistance > minFriendDistance) continue;

                ////дальше от слэшеров
                //index = 0;
                //while (index < minDiPath.Count)
                //{
                //    if (di.MinSlashersDist > minDamageItem.MinSlashersDist)
                //    {
                //        isFinished = true;
                //        minDamageItem = di;
                //        minFds = fds;
                //        minFriendDistance = friendDistance;
                //        nextMinFd = nextFd;
                //        minOrderedWanderers = orderedWanderers;
                //        break;
                //    }

                //    if (di.MinSlashersDist < minDamageItem.MinSlashersDist)
                //    {
                //        isFinished = true;
                //        break;
                //    }

                //    index++;
                //}
                //if (isFinished) continue;

                //меньше новых странников
                index = 0;
                while (index < minDiPath.Count)
                {
                    if (di.NewWanderersCount < minDamageItem.NewWanderersCount)
                    {
                        isFinished = true;
                        minDamageItem = di;
                        minFds = fds;
                        minFriendDistance = friendDistance;
                        nextMinFd = nextFd;
                        minOrderedWanderers = orderedWanderers;
                        break;
                    }

                    if (di.NewWanderersCount > minDamageItem.NewWanderersCount)
                    {
                        isFinished = true;
                        break;
                    }

                    index++;
                }
                if (isFinished) continue;

                ////меньше слэшеров
                //index = 0;
                //while (index < minDiPath.Count)
                //{
                //    if (di.MinSlashersDistCount < minDamageItem.MinSlashersDistCount)
                //    {
                //        isFinished = true;
                //        minDamageItem = di;
                //        minFds = fds;
                //        minFriendDistance = friendDistance;
                //        nextMinFd = nextFd;
                //        minOrderedWanderers = orderedWanderers;
                //        break;
                //    }

                //    if (di.MinSlashersDistCount > minDamageItem.MinSlashersDistCount)
                //    {
                //        isFinished = true;
                //        break;
                //    }

                //    index++;
                //}
                //if (isFinished) continue;

                //следующий ближайший друг
                if (nextFd < nextMinFd)
                {
                    minDamageItem = di;
                    minFds = fds;
                    minFriendDistance = friendDistance;
                    nextMinFd = nextFd;
                    minOrderedWanderers = orderedWanderers;
                    continue;
                }
                if (nextFd > nextMinFd) continue;

                //наименьший суммарный урон с учетом врага
                if (di.SumDamage < minDamageItem.SumDamage)
                {
                    minDamageItem = di;
                    minFds = fds;
                    minFriendDistance = friendDistance;
                    nextMinFd = nextFd;
                    minOrderedWanderers = orderedWanderers;
                    continue;
                }
                if (di.SumDamage > minDamageItem.SumDamage) continue;

                //наиболее дальний странник

                for (int j = 0; j < orderedWanderers.Count; ++j)
                {
                    var w = orderedWanderers[j];
                    var minW = minOrderedWanderers[j];
                    if (Pathes[w.X, w.Y][di.Point.X, di.Point.Y].Count >
                        Pathes[minW.X, minW.Y][minDamageItem.Point.X, minDamageItem.Point.Y].Count)
                    {
                        isFinished = true;
                        minDamageItem = di;
                        minFds = fds;
                        minFriendDistance = friendDistance;
                        nextMinFd = nextFd;
                        minOrderedWanderers = orderedWanderers;
                        break;
                    }
                    if (Pathes[w.X, w.Y][di.Point.X, di.Point.Y].Count <
                             Pathes[minW.X, minW.Y][minDamageItem.Point.X, minDamageItem.Point.Y].Count)
                    {
                        isFinished = true;
                        break;
                    }
                }
                if (isFinished) continue;
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
            IList<Explorer> allExplorers,
            IList<Wanderer> wanderers,
            bool canBeYelled)
        {
            if (myExplorer.Plans == 0) return false;
            if (CurrentPlanCooldown > 0 || CurrentLightCooldown > 0) return false;

            if (canBeYelled)
                foreach (var w in wanderers)
                {
                    var targetE = allExplorers.SingleOrDefault(e => e.Id == w.TargetId);
                    if (targetE != null && targetE.X == myExplorer.X && targetE.Y == myExplorer.Y)
                    {
                        if (Pathes[w.X, w.Y][targetE.X, targetE.Y].Count <= 3) return false;//TODO: забить, когда мало здоровья
                    }
                }

            var closeExplorersCount = allExplorers.Count(
                e => e.Id != myExplorer.Id && Pathes[myExplorer.X, myExplorer.Y][e.X, e.Y].Count - 1 <= PlanDist);

            var restoringSanity = (PlanAddSanity + closeExplorersCount * PlanAddSanity) * PlanCooldown;
            var neededSanity = StartSanity - myExplorer.Sanity;//TODO: consider sanity loss
            if (restoringSanity > neededSanity) return false;

            if (closeExplorersCount < UsePlanExplorersCount && myExplorer.Sanity > CriticalPlanSanity)
                return false;

            return true;
        }

        static bool NewUseYell(Explorer myExplorer, IList<Explorer> allExplorers, IList<Wanderer> wanderers,
            IList<Wanderer> slashers)
        {
            var newExplorers = new List<Explorer>();
            var yelledExplorers = new List<Explorer>();


            foreach (var e in allExplorers)
            {
                if (e.Id != myExplorer.Id && !YelledIds.Contains(e.Id) && GetManhattenDist(myExplorer, e) <= YellDist && e.YellTime == 0)
                {
                    yelledExplorers.Add(e);
                    newExplorers.Add(new Explorer(e.Id, e.X, e.Y, e.Sanity, e.Plans, e.Lights) { YellTime = YellStanTime });
                }
                else
                {
                    newExplorers.Add(e);
                }
            }

            if (!yelledExplorers.Any()) return false;

            var maxPossDamage = 0;
            foreach (var e in yelledExplorers)
            {
                var damage = 0;
                foreach (var w in wanderers)
                {
                    var dist = Pathes[w.X, w.Y][e.X, e.Y].Count - 1;
                    if (dist == 2) damage++;
                    else if (dist == 1 && GetManhattenDist(myExplorer, w) > 1) damage++;
                }

                foreach (var s in slashers)
                {
                    if (IsVisblePoint(s, e.X, e.Y)) damage++;
                }

                if (damage > maxPossDamage) maxPossDamage = damage;
            }
            if (maxPossDamage < YellExplorersDamage) return false;

            var yelledIds = yelledExplorers.Select(e => e.Id).ToList();

            var sourcePoint = PointsTable[myExplorer.X, myExplorer.Y];
            var di = GetDamageItemsRec(myExplorer, newExplorers, wanderers, slashers, sourcePoint, 0, yelledIds);
            var useYell = di[0].YelledEplorersSumDamage - di[0].MySumDamage >= YellExplorersDamage;
            if (useYell) YelledIds.AddRange(yelledIds);
            return useYell;
        }

        static bool CanBeYelled(Explorer myExplorer, IList<Explorer> allExplorers)
        {
            foreach (var e in allExplorers)
            {
                if (e.Id == myExplorer.Id) continue;
                if (YelledMeIds.Contains(e.Id)) continue;
                if (GetManhattenDist(myExplorer, e) > YellDist) continue;
                return true;
            }

            return false;
        }

        static bool NewUseLight(Explorer myExplorer,
            IList<Explorer> allExplorers,
            IList<Wanderer> wanderers,
            bool canBeYelled)
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
                    if (canBeYelled && Pathes[wanderer.X, wanderer.Y][targetExplorer.X, targetExplorer.Y].Count <= 3) return false;
                    myExplorerTargetList.Add(wanderer);
                }
            }

            var myExplorerPoint = PointsTable[myExplorer.X, myExplorer.Y];
            var lightedPoints = GetLightedPoints(myExplorerPoint, Points);
            foreach (var p in lightedPoints)
                p.Weight += LightAddDist;

            var useLight = false;
            foreach (var wanderer in myExplorerTargetList)
            {
                var startPoint = PointsTable[wanderer.X, wanderer.Y];
                var myPath = Calculator.GetPath(startPoint, myExplorerPoint, Points);
                var myDist = myPath.Sum(p => (p as Point).Weight);


                foreach (var explorer in allExplorers)
                {
                    if (explorer.X == myExplorer.X && explorer.Y == myExplorer.Y) continue;

                    var finalPoint = PointsTable[explorer.X, explorer.Y];
                    var path = Calculator.GetPath(startPoint, finalPoint, Points);
                    var dist = path.Sum(p => (p as Point).Weight);

                    if (myDist <= dist) continue;
                    if (path.Count <= 3)
                    {
                        useLight = true;
                        break;
                    };

                    Point point2 = path[2] as Point;
                    var afterLightMyPath = Pathes[point2.X, point2.Y][myExplorer.X, myExplorer.Y];
                    var afterLightExplorerPath = Pathes[point2.X, point2.Y][explorer.X, explorer.Y];

                    if (afterLightMyPath.Count >
                        afterLightExplorerPath.Count)
                    {
                        useLight = true;
                        break;
                    }
                }
            }
            foreach (var p in lightedPoints)
                p.Weight = 1;
            return useLight;
        }

        //TODO: consider slashers

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
            var damageItems = GetDamageItemsRec(myExplorer, explorers, wanderers, slashers, ExplorerPoints[myExplorer.Id], 0, new List<int>());
            foreach (var di in damageItems)
            {
                Console.Error.WriteLine($"x:{di.Point.X} y:{di.Point.Y} SumD:{di.MySumDamage} D:{di.MyDamage} WD:{di.MinWandererDist} NWC:{di.NewWanderersCount} WDC:{di.MinWandererDistCount} ?:{di.SumDamage}");
            }

            return damageItems;

        }

        private static bool CanHide(Wanderer s, Point p)
        {
            if (!IsVisblePoint(s, p.X, p.Y)) return true;
            foreach (Point n in p.Neighbours)
            {
                if (n.Weight == BigValue) continue;
                if (!IsVisblePoint(s, n.X, n.Y))
                {
                    return true;
                }
            }

            return false;
        }

        private static IList<DamageItem> GetDamageItemsRec(Explorer myExplorer, IList<Explorer> explorers,
            IList<Wanderer> wanderers, IList<Wanderer> slashers, Point sourcePoint, int depth, IEnumerable<int> yelledIds)
        {
            var damageItems = new List<DamageItem>();

            var currentPossibleMoves = GetCurrentPossibleMoves(sourcePoint, depth == 0 && yelledIds.Any());
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

                var minEnemyDamage = int.MaxValue;
                foreach (var e in newExplorers.Where(e => e.Id != myExplorer.Id))
                {
                    var eDamage = newWanderes.Count(w => w.X == e.X && w.Y == e.Y);
                    eDamage += newSlashers.Count(s => s.State == 4 && s.Time == 6 && s.X == e.X && s.Y == e.Y);
                    if (eDamage < minEnemyDamage) minEnemyDamage = eDamage;
                }

                int yelledExplorersDamage = 0;
                foreach (var e in explorers.Where(e => e.Id != myExplorer.Id && e.YellTime > 0 && yelledIds.Contains(e.Id)))
                {
                    var eDamage = newWanderes.Count(w => w.X == e.X && w.Y == e.Y);
                    eDamage += newSlashers.Count(s => s.State == 4 && s.Time == 6 && s.X == e.X && s.Y == e.Y);
                    if (eDamage > yelledExplorersDamage) yelledExplorersDamage = eDamage;
                }

                int minWandererDist = int.MaxValue;
                int minWandererDistCount = 0;
                var newWanderersCount = 0;
                foreach (var w in wanderers)//старые позиции
                {
                    var dist = Pathes[w.X, w.Y][move.X, move.Y].Count;
                    if (dist < minWandererDist)
                    {
                        minWandererDist = dist;
                        minWandererDistCount = 1;
                    }
                    else if (dist == minWandererDist)
                    {
                        minWandererDistCount++;
                    }

                    if (w.TargetId == myExplorer.Id) continue;
                    var isMinDist = true;
                    foreach (var e in newExplorers.Where(e => e.Id != myExplorer.Id))//старые позиции, т.к. не уверены, куда они пойдут
                    {
                        var eDist = Pathes[w.X, w.Y][e.X, e.Y].Count;
                        if (eDist < dist)
                        {
                            isMinDist = false;
                            break;
                        }
                    }

                    if (isMinDist) newWanderersCount++;
                }

                foreach (var w in newSlashers)
                {
                    var dist = Pathes[w.X, w.Y][move.X, move.Y].Count;
                    if (dist < minWandererDist)
                    {
                        minWandererDist = dist;
                        minWandererDistCount = 1;
                    }
                    else if (dist == minWandererDist)
                    {
                        minWandererDist++;
                    }

                    if (w.TargetId == myExplorer.Id || w.State != 0 && w.State != 1 && w.State != 4) continue;
                    var isMinDist = true;
                    foreach (var e in newExplorers.Where(e => e.Id != myExplorer.Id))//старые позиции, т.к. не уверены, куда они пойдут
                    {
                        var eDist = Pathes[w.X, w.Y][e.X, e.Y].Count;
                        if (eDist < dist)
                        {
                            isMinDist = false;
                            break;
                        }
                    }

                    if (isMinDist) newWanderersCount++;
                }

                var canBeYelled = newExplorers.Any(e =>
                    e.Id != myExplorer.Id && !YelledMeIds.Contains(e.Id) &&
                    GetManhattenDist(e, myCurrExplorer) <= YellDist);

                var damageItem = new DamageItem()
                {
                    MyDamage = damage,
                    Damage = damage - minEnemyDamage,
                    Point = move,
                    MinWandererDist = minWandererDist,
                    MinWandererDistCount = minWandererDistCount,
                    CanBeYelled = canBeYelled,
                    NewWanderersCount = newWanderersCount,
                    YelledExplorersDamage = yelledExplorersDamage
                };

                if (depth == PredictionDepth)
                {
                    damageItem.DamageChildren = new List<DamageItem>();
                    //прибавляем урон, если нас заперли
                    var minNearWanderers = int.MaxValue;
                    foreach (Point n in move.Neighbours)
                    {
                        if (n.Weight == BigValue) continue;
                        var wCount = newWanderes.Count(w => w.X == n.X && w.Y == n.Y);
                        if (wCount < minNearWanderers)
                        {
                            minNearWanderers = wCount;
                        }
                    }

                    var rushingSlashers = 0;
                    foreach (var s in newSlashers)
                    {
                        if (s.State != 3) continue;
                        if (s.TargetId == myExplorer.Id)
                        {
                            if (!CanHide(s, move)) rushingSlashers++;
                        }
                        else
                        {
                            var targetE = newExplorers.SingleOrDefault(e => e.Id == s.Id);
                            if (targetE != null && CanHide(s, PointsTable[targetE.X, targetE.Y]))
                            {
                                var nearestVisibleE = GetNearestExplorer(s, newExplorers, true);
                                if (nearestVisibleE != null && nearestVisibleE.Id == myExplorer.Id) rushingSlashers++;
                            }
                        }

                        //if (!IsVisblePoint(s, move.X, move.Y)) continue;
                        //var canHide = false;
                        //foreach (Point n in move.Neighbours)
                        //{
                        //    if (n.Weight == BigValue) continue;
                        //    if (!IsVisblePoint(s, n.X, n.Y))
                        //    {
                        //        canHide = true;
                        //        break;
                        //    }
                        //}

                        //if (!canHide) rushingSlashers++;
                    }

                    damageItem.Damage += (minNearWanderers + rushingSlashers);
                    damageItem.MyDamage += (minNearWanderers + rushingSlashers);
                    continue;
                }


                newWanderes = newWanderes.Where(w => !newExplorers.Any(s => GetManhattenDist(w, s) == 0)).ToList();//удаляем странников, дошедших до цели
                var nextDamages = GetDamageItemsRec(myExplorer, newExplorers, newWanderes, newSlashers, move, depth + 1, yelledIds);
                damageItem.DamageChildren = nextDamages;

                damageItems.Add(damageItem);
            }

            return damageItems;

        }

        private static IList<Point> GetCurrentPossibleMoves(Point sourcePoint, bool isYelling)
        {
            var possibleMoves = new List<Point>() { sourcePoint };
            if (isYelling) return possibleMoves;
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

                if (explorer.YellTime > 0)
                {
                    var yelledExplorer = new Explorer(explorer.Id, explorer.X, explorer.Y, explorer.Sanity, explorer.Plans,
                        explorer.Lights)
                    { YellTime = explorer.YellTime - 1 };
                    newExplorers.Add(yelledExplorer);
                    continue;
                }

                var explorerPoint = PointsTable[explorer.X, explorer.Y];
                var neighbours = explorerPoint.GetNeighbors(Points);
                var minDamagePoint = explorerPoint;
                var minDamage = 0;
                int minVisibleSlashers = 0;
                int minPreparingRushSlashers = 0;
                var minWanderersDist = int.MaxValue;
                int minExplorerDist = int.MaxValue;
                foreach (var e in explorers.Where(e => e.Id != explorer.Id))
                {
                    var path = Pathes[explorer.X, explorer.Y][e.X, e.Y];
                    var dist = path.Count;
                    if (dist < minExplorerDist) minExplorerDist = dist;
                }

                foreach (var w in wanderers)
                {
                    var dist = Pathes[w.X, w.Y][explorer.X, explorer.Y].Count;
                    if (dist <= 2)
                        minDamage++;
                    if (dist < minWanderersDist)
                        minWanderersDist = dist;
                }

                foreach (var s in slashers)
                {
                    if (s.TargetId != explorer.Id) continue;
                    var dist = Pathes[s.X, s.Y][explorer.X, explorer.Y].Count;
                    if (dist < minWanderersDist)
                        minWanderersDist = dist;
                }

                foreach (var s in slashers)
                {
                    var isVisible = IsVisblePoint(s, explorer.X, explorer.Y);
                    if (isVisible && s.State == 3) minDamage++;
                    if (!isVisible) continue;
                    minVisibleSlashers++;

                    if (s.State != 3 && s.State != 2 && s.State != 0 || s.Time > 2) continue;
                    //var targetExplorer = explorers.SingleOrDefault(e => e.Id == s.TargetId);
                    //if (targetExplorer != null && targetExplorer.X == explorer.X && targetExplorer.Y == explorer.Y)
                    minPreparingRushSlashers++;
                }

                foreach (Point n in neighbours)
                {
                    if (n.Weight == BigValue) continue;
                    var damage = 0;
                    var wanderersDist = int.MaxValue;

                    int explorerDist = int.MaxValue;
                    foreach (var e in explorers.Where(e => e.Id != explorer.Id))
                    {
                        var path = Pathes[n.X, n.Y][e.X, e.Y];
                        var dist = path.Count;
                        if (dist < explorerDist) explorerDist = dist;
                    }

                    foreach (var w in wanderers)
                    {
                        var dist = Pathes[w.X, w.Y][n.X, n.Y].Count;
                        if (dist <= 1)
                            damage++;
                        if (dist < wanderersDist)
                            wanderersDist = dist;
                    }

                    foreach (var s in slashers)
                    {
                        if (s.TargetId != explorer.Id) continue;
                        var dist = Pathes[s.X, s.Y][n.X, n.Y].Count;
                        if (dist < wanderersDist)
                            wanderersDist = dist;
                    }

                    var visibleSlashers = 0;
                    var preparingRushSlashers = 0;
                    foreach (var s in slashers)
                    {
                        var isVisible = IsVisblePoint(s, n.X, n.Y);
                        if (isVisible && s.State == 3) damage++;
                        if (!isVisible) continue;
                        visibleSlashers++;

                        if (s.State != 3 && s.State != 2 && s.State != 0 || s.Time > 2) continue;
                        //var targetExplorer = explorers.SingleOrDefault(e => e.Id == s.TargetId);
                        //if (targetExplorer != null && targetExplorer.X == n.X && targetExplorer.Y == n.Y)
                        preparingRushSlashers++;
                    }

                    //var visibleSlashers = slashers.Count(s => IsVisblePoint(s, n.X, n.Y));
                    //foreach (var slasher in slashers.Where(s => s.State == 3))
                    //{
                    //    if (IsVisblePoint(slasher, n.X, n.Y)) damage++;
                    //}

                    if (damage < minDamage)
                    {
                        minDamage = damage;
                        minDamagePoint = n;
                        minWanderersDist = wanderersDist;
                        minVisibleSlashers = visibleSlashers;
                        minExplorerDist = explorerDist;
                        minPreparingRushSlashers = preparingRushSlashers;
                        continue;
                    }
                    if (damage > minDamage) continue;

                    if (preparingRushSlashers < minPreparingRushSlashers)
                    {
                        minDamage = damage;
                        minDamagePoint = n;
                        minWanderersDist = wanderersDist;
                        minVisibleSlashers = visibleSlashers;
                        minExplorerDist = explorerDist;
                        minPreparingRushSlashers = preparingRushSlashers;
                        continue;
                    }
                    if (preparingRushSlashers > minPreparingRushSlashers) continue;

                    if (wanderersDist > minWanderersDist)
                    {
                        minDamage = damage;
                        minDamagePoint = n;
                        minWanderersDist = wanderersDist;
                        minVisibleSlashers = visibleSlashers;
                        minExplorerDist = explorerDist;
                        minPreparingRushSlashers = preparingRushSlashers;
                        continue;
                    }
                    if (wanderersDist < minWanderersDist) continue;

                    if (visibleSlashers < minVisibleSlashers)
                    {
                        minDamage = damage;
                        minDamagePoint = n;
                        minWanderersDist = wanderersDist;
                        minVisibleSlashers = visibleSlashers;
                        minExplorerDist = explorerDist;
                        minPreparingRushSlashers = preparingRushSlashers;
                        continue;
                    }
                    if (visibleSlashers > minVisibleSlashers) continue;

                    if (explorerDist < minExplorerDist)
                    {
                        minDamage = damage;
                        minDamagePoint = n;
                        minWanderersDist = wanderersDist;
                        minVisibleSlashers = visibleSlashers;
                        minExplorerDist = explorerDist;
                        minPreparingRushSlashers = preparingRushSlashers;
                        continue;
                    }
                    if (explorerDist < minExplorerDist) continue;


                }
                var newExplorer = new Explorer(explorer.Id, minDamagePoint.X, minDamagePoint.Y, explorer.Sanity, explorer.Plans, explorer.Lights);
                newExplorers.Add(newExplorer);
            }

            return newExplorers;
        }


        private static IList<Wanderer> GetNewWanderes(IList<Explorer> explorers, IList<Wanderer> wanderers)
        {
            var newWanderes = new List<Wanderer>();
            foreach (var wanderer in wanderers)
            {
                if (wanderer.State == 0)
                {
                    var newTime = wanderer.Time - 1;
                    var newState = newTime == 0 ? 1 : 0;
                    newWanderes.Add(new Wanderer(wanderer.Id, wanderer.X, wanderer.Y, newTime, newState, wanderer.TargetId));
                    continue;
                }
                if (wanderer.Time == 0) continue;


                var minPathCount = int.MaxValue;
                Explorer targetExplorer = null;
                foreach (var e in explorers)
                {
                    var dist = Pathes[wanderer.X, wanderer.Y][e.X, e.Y].Count;
                    if (dist < minPathCount || dist == minPathCount && e.Id == wanderer.TargetId)
                    {
                        minPathCount = dist;
                        targetExplorer = e;
                    }
                }

                Point nearestPoint = PointsTable[wanderer.X, wanderer.Y];

                int minDist = Pathes[wanderer.X, wanderer.Y][targetExplorer.X, targetExplorer.Y].Count;
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
                        var dist = Pathes[n.X, n.Y][targetExplorer.X, targetExplorer.Y].Count;

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

        private static Explorer GetNearestExplorer(Wanderer slasher, IList<Explorer> explorers, bool needVisible)
        {
            Explorer explorer = null;
            var minDist = int.MaxValue;
            foreach (var e in explorers)
            {
                if (needVisible && !IsVisblePoint(slasher, e.X, e.Y)) continue;
                var dist = Pathes[slasher.X, slasher.Y][e.X, e.Y].Count;
                if (dist < minDist)
                {
                    minDist = dist;
                    explorer = e;
                }
            }
            return explorer;
        }


        private static IList<Wanderer> GetNewSlashers(IList<Explorer> explorers, IList<Wanderer> slashers)
        {
            var newSlashers = new List<Wanderer>();
            foreach (var slasher in slashers)
            {
                var targetExplorer = explorers.SingleOrDefault(e => e.Id == slasher.TargetId);

                if (slasher.State == 0) //зарождается
                {
                    var newTime = slasher.Time - 1;
                    int newState;
                    int newTargetExplorerId;

                    if (newTime > 0)
                    {
                        newState = 0;
                        newTargetExplorerId = slasher.TargetId;
                    }
                    else
                    {
                        if (targetExplorer != null)
                        {
                            if (IsVisblePoint(slasher, targetExplorer.X, targetExplorer.Y))
                            {
                                newState = 3;
                                newTime = 0;
                                newTargetExplorerId = slasher.TargetId;
                            }
                            else
                            {
                                var nearestVisibleExporer = GetNearestExplorer(slasher, explorers, true);
                                if (nearestVisibleExporer != null)
                                {
                                    newState = 3;
                                    newTime = 0;
                                    newTargetExplorerId = nearestVisibleExporer.Id;
                                }
                                else
                                {
                                    newState = 1;
                                    newTime = -1;
                                    newTargetExplorerId = slasher.TargetId;
                                }
                            }
                        }
                        else
                        {
                            newState = 1;
                            newTime = -1;
                            newTargetExplorerId = -1;
                        }
                    }
                    var newSlasher = new Wanderer(slasher.Id, slasher.X, slasher.Y, newTime, newState, newTargetExplorerId);
                    newSlashers.Add(newSlasher);
                }
                else if (slasher.State == 1)//странствует
                {
                    if (targetExplorer == null)
                    {
                        int newTargetId;
                        int newState;
                        int newTime;
                        var visibleNearestExplorer = GetNearestExplorer(slasher, explorers, true);
                        if (visibleNearestExplorer != null)
                        {
                            newTargetId = visibleNearestExplorer.Id;
                            newState = 2;
                            newTime = 2;
                        }
                        else
                        {
                            var nearestExplorer = GetNearestExplorer(slasher, explorers, false);
                            newTargetId = nearestExplorer.Id;
                            newState = 1;
                            newTime = slasher.Time;
                        }
                        var newSlasher = new Wanderer(slasher.Id, slasher.X, slasher.Y, newTime, newState, newTargetId);
                        newSlashers.Add(newSlasher);
                        continue;
                    }

                    var minDist = int.MaxValue;
                    Explorer minDistExplorer = null;
                    var isVisible = false;
                    foreach (var e in explorers)
                    {
                        var path = Pathes[slasher.X, slasher.Y][e.X, e.Y];
                        var isCurrVisible = IsVisblePoint(slasher, e.X, e.Y);
                        var dist = path.Count;
                        if (dist < minDist)
                        {
                            minDist = dist;
                            minDistExplorer = e;
                            isVisible = isCurrVisible;
                        }
                        else if (dist == minDist && !isVisible && isCurrVisible)
                        {
                            minDistExplorer = e;
                            isVisible = true;
                        }


                    }


                    //var explorerDists = new Dictionary<Explorer, int>();
                    //foreach (var e in explorers)
                    //{
                    //    var path = Pathes[slasher.X, slasher.Y][e.X, e.Y];
                    //    var dist = path.Count;
                    //    explorerDists.Add(e, dist);
                    //}

                    //var minDist = explorerDists.Values.Min();
                    //var nearestExplorers = explorerDists.Keys.Where(e => explorerDists[e] == minDist);
                    //var visibleExplorers = nearestExplorers.Where(e => IsVisblePoint(slasher, e.X, e.Y));

                    //var nearestVisibleExplorer = visibleExplorers.FirstOrDefault();
                    if (isVisible)
                    {
                        var newSlasher = new Wanderer(slasher.Id, slasher.X, slasher.Y, 2, 2, minDistExplorer.Id);
                        newSlashers.Add(newSlasher);
                    }
                    else
                    {
                        var path = Pathes[slasher.X, slasher.Y][minDistExplorer.X, minDistExplorer.Y];

                        var x = path.Count == 1 ? minDistExplorer.X : (path[1] as Point).X;
                        var y = path.Count == 1 ? minDistExplorer.Y : (path[1] as Point).Y;

                        var newSlasher = new Wanderer(slasher.Id, x, y, slasher.Time, 1, minDistExplorer.Id);
                        newSlashers.Add(newSlasher);
                    }

                }
                else if (slasher.State == 2) //готовится к прыжку
                {
                    var newTime = slasher.Time - 1;
                    var newState = newTime == 0 ? 3 : 2;
                    int newTargetExplorerId = -1;

                    if (targetExplorer != null)
                    {
                        if (IsVisblePoint(slasher, targetExplorer.X, targetExplorer.Y))
                        {
                            newTargetExplorerId = targetExplorer.Id;
                        }
                        else
                        {
                            var visibleNearestExplorer = GetNearestExplorer(slasher, explorers, true); //TODO
                            if (visibleNearestExplorer != null)
                            {
                                newTargetExplorerId = visibleNearestExplorer.Id;
                            }
                        }
                    }

                    var newSlasher = new Wanderer(slasher.Id, slasher.X, slasher.Y, newTime, newState, newTargetExplorerId);
                    newSlashers.Add(newSlasher);

                }
                else if (slasher.State == 3) //прыгает
                {
                    int newSlasherX, newSlasherY;
                    if (targetExplorer != null)
                    {
                        if (IsVisblePoint(slasher, targetExplorer.X, targetExplorer.Y))
                        {
                            newSlasherX = targetExplorer.X;
                            newSlasherY = targetExplorer.Y;
                        }
                        else
                        {
                            var visibleNearestExplorer = GetNearestExplorer(slasher, explorers, true); //TODO
                            if (visibleNearestExplorer != null)
                            {
                                newSlasherX = visibleNearestExplorer.X;
                                newSlasherY = visibleNearestExplorer.Y;
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
                        var visibleNearestExplorer = GetNearestExplorer(slasher, explorers, true); //TODO
                        if (visibleNearestExplorer != null)
                        {
                            newSlasherX = visibleNearestExplorer.X;
                            newSlasherY = visibleNearestExplorer.Y;
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

                else if (slasher.State == 4)//застанен
                {
                    if (slasher.Time > 0)
                    {
                        var newSlasher = new Wanderer(slasher.Id, slasher.X, slasher.Y, slasher.Time - 1, slasher.State, slasher.TargetId);
                        newSlashers.Add(newSlasher);
                    }
                    else
                    {
                        var newSlasher = new Wanderer(slasher.Id, slasher.X, slasher.Y, -1, 1, slasher.TargetId);
                        newSlashers.Add(newSlasher);
                    }
                }



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

        private static List<int> YelledIds = new List<int>();
        private static IList<int> YelledMeIds = new List<int>();

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

        private const int YellExplorersDamage = 2;//TODO: up!

        private const int LightAddDist = 4;
        private const int LightRange = 5;
        private const int LightCooldown = 3;

        private const int FriendAuraRange = 2;

        private const int ShelterDefaultEnergy = 10;
        private const int MaxShelterEnergyLeak = 3;

        private const int ManhDistToUseAStar = 4;
        private const int PredictionDepth = 4;

        //private static IDictionary<APoint, ExpansionMatrixConteiner> Ems;
        //private static IDictionary<Point, IDictionary<Point, IList<APoint>>> Pathes;
        private static IList<APoint>[,][,] Pathes;//массив путей из каждой точки сетки в каждую
        //private static int[,][,] LightedLengths;//длины путей из каждой точке в каждую, если в источнике включить свет

        #endregion

    }
}

