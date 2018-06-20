using System;
using System.CodeDom;
using System.Collections.Generic;
using System.Linq;
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

            public override IEnumerable<APoint> GetNeighbors(IEnumerable<APoint> apoints)
            {
                var resPoints = new List<Point>();
                foreach (var apoint in apoints)
                {
                    var point = (Point)apoint;
                    if (point.Y == Y && (point.X == X - 1 || point.X == X + 1))
                    {
                        resPoints.Add(point);
                    }
                    else if (point.X == X && (point.Y == Y - 1 || point.Y == Y + 1))
                    {
                        resPoints.Add(point);
                    }
                }
                return resPoints;
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
            private static IDictionary<APoint, ExpansionMatrixConteiner> GetExpansionMatrices(IEnumerable<APoint> startPoints, IEnumerable<APoint> allPoints)
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

            private static IList<APoint> ReconstructPath(APoint goal, ExpansionMatrixConteiner expansionMatrixConteiner, IEnumerable<APoint> allPoints)
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
                res.Add(new Point() { X = point.X, Y = point.Y, Weight = point.Weight });
            return res;
        }

        static void Main(string[] args)
        {
            string[] inputs;
            int width = int.Parse(Console.ReadLine());
            //Console.Error.WriteLine(width);
            int height = int.Parse(Console.ReadLine());
            //Console.Error.WriteLine(height);

            Points = new List<Point>();
            Point startPoint = null, finalPoint = null;

            for (int i = 0; i < height; i++)
            {
                string line = Console.ReadLine();
                //Console.Error.WriteLine(line);
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

                }
            }

            var str = Console.ReadLine();
            //Console.Error.WriteLine(str);
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

            // game loop
            while (true)
            {
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
                //Console.Error.WriteLine(str);
                int entityCount = int.Parse(str); // the first given entity corresponds to your explorer
                for (int i = 0; i < entityCount; i++)
                {
                    str = Console.ReadLine();
                    //Console.Error.WriteLine(str);
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
                        ExplorerPoints.Add(explorer.Id, Points.Single(p => p.X == explorer.X && p.Y == explorer.Y));
                    }
                    else if (entityType == "WANDERER")
                    {
                        var wanderer = new Wanderer(id, x, y, param0, param1, param2);
                        wanderers.Add(wanderer);
                        WandererPoints.Add(wanderer.Id, Points.Single(p => p.X == wanderer.X && p.Y == wanderer.Y));
                    }
                    else if (entityType == "SLASHER")
                    {
                        var slaher = new Wanderer(id, x, y, param0, param1, param2);
                        slashers.Add(slaher);
                        SlasherPoints.Add(slaher.Id, Points.Single(p => p.X == slaher.X && p.Y == slaher.Y));
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

                UpdateSlasherTargets(slashers);

                var slasherDangerousPoints = GetSlasherDangerousPoints(allExplorers, slashers, points, myExplorer);
                var wandererDangerousPoints = GetWandererDangerousPoints(myExplorer, wanderers, points);

                //var isMyCellDangerous = slasherDangerousPoints.Keys.Any(p => p.X == myExplorer.X && p.Y == myExplorer.Y) ||
                //                        wandererDangerousPoints.Keys.Any(p => p.X == myExplorer.X && p.Y == myExplorer.Y);


                foreach (var sdp in slasherDangerousPoints.Keys)
                {
                    //if (sdp.X == myExplorer.X && sdp.Y == myExplorer.Y)
                    //    continue; //не помечаем свою ячейку неппроходимой
                    sdp.Weight += SlasherWeight * slasherDangerousPoints[sdp];
                }

                foreach (var wdp in wandererDangerousPoints.Keys)
                {
                    //if (wdp.X == myExplorer.X && wdp.Y == myExplorer.Y)
                    //    continue; //не помечаем свою ячейку непроходимой
                    wdp.Weight += WandererWeight * wandererDangerousPoints[wdp];
                }

                var goPoint = GetGoPoint(myExplorer, allExplorers, shelters, points);
                finalPoint = points.Single(p => p.X == goPoint.X && p.Y == goPoint.Y);
                var path = Calculator.GetPath(startPoint, finalPoint, points);

                //Console.Error.WriteLine($"{startPoint.X} {startPoint.Y} {startPoint.Weight}");
                //foreach (Point neighbour in startPoint.GetNeighbors(points))
                //    if (neighbour.Weight < BigValue)
                //        Console.Error.WriteLine($"{neighbour.X} {neighbour.Y} {neighbour.Weight}");

                
                var escapePoint = GetEscapePoint(startPoint, points, allExplorers, wanderers, slashers, myExplorer.Id);
                var isBetterToHide = path.Count > 1 && escapePoint.Weight < (path[1] as Point).Weight;//опасно идти за эксплорером

                var auraRange = goPoint is Shelter ? 0 : AuraRange;
                var goPointDist = GetManhattenDist(myExplorer, goPoint);
                if (goPointDist <= auraRange || isBetterToHide)
                {
                    var isCurrPointSafe = startPoint.Weight <= escapePoint.Weight;
                    //TODO: из-за isCurrPointSafe shouldStay сработает, даже если выгоднее другая точка
                    var shouldStay = escapePoint.X == myExplorer.X && escapePoint.Y == myExplorer.Y;

                   

                    if (shouldStay || isCurrPointSafe)
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
                            Console.WriteLine("PLAN");
                            continue;
                        }

                        if (useLight)
                        {
                            Console.WriteLine("LIGHT");
                            continue;
                        }
                    }
                    
                    if (shouldStay)
                    {
                        //Console.Error.WriteLine("Curr point is most safety");
                        Console.WriteLine("WAIT");
                    }
                    else
                    {
                        //Console.Error.WriteLine("Time to run");
                        Console.WriteLine("MOVE " + escapePoint.X + " " + escapePoint.Y);
                    }
                }
                else
                {
                    var step = path[1] as Point;
                    Console.WriteLine("MOVE " + step.X + " " + step.Y);
                }
            }
        }

        static int GetWanderesCount(int x, int y, int myExplorerId, IList<Explorer> explorers, IList<Wanderer> wanderers)
        {
            var wanderesCount = 0;
            var destPoint = Points.Single(p => p.X == x && p.Y == y);
            foreach (var wanderer in wanderers.Where(w => w.State == 1))
            {
                if (!WandererExplorerPathDistances.ContainsKey(wanderer.Id))
                    WandererExplorerPathDistances.Add(wanderer.Id, new Dictionary<int, int>());

                var startPoint = WandererPoints[wanderer.Id];
                var path = Calculator.GetPath(startPoint, destPoint, Points);
                var destDist = path.Count;
                var isCloser = false;
                foreach (var explorer in explorers.Where(e => e.Id != myExplorerId))
                {
                    if (!WandererExplorerPathDistances[wanderer.Id].ContainsKey(explorer.Id))
                    {
                        var finalPoint = ExplorerPoints[explorer.Id];
                        var explorerPath = Calculator.GetPath(startPoint, finalPoint, Points);
                        WandererExplorerPathDistances[wanderer.Id].Add(explorer.Id, explorerPath.Count);
                    }
                    var dist = WandererExplorerPathDistances[wanderer.Id][explorer.Id];
                    if (dist < destDist || dist == destDist && wanderer.TargetId == explorer.Id)
                    {
                        isCloser = true;
                        break;
                    }


                }
                if (!isCloser) wanderesCount++;
            }

            return wanderesCount;
        }

        /// <summary>
        /// Ишет точку с минимальным весом, куда надо бежать из текущей
        /// Если веса двух точек одинаковы, бежит в ту, которая ближе к другому Explorer'у
        /// Не проверяет текущую точку на возможность остаться в ней
        /// </summary>
        /// <param name="startPoint"></param>
        /// <param name="points"></param>
        /// <param name="allExplorers"></param>
        /// <param name="wanderers"></param>
        /// <returns></returns>
        static Point GetEscapePoint(Point startPoint, IList<Point> points, IList<Explorer> allExplorers, IList<Wanderer> wanderers, IList<Wanderer> slashers, int myExplorerId)
        {
            var neighbours = startPoint.GetNeighbors(points);

            Point minWeightPoint = startPoint;
            var minWeight = startPoint.Weight;
            var minExplorerDist = allExplorers.Where(e => e.Id != myExplorerId).Min(e => GetManhattenDist(e.X, e.Y, startPoint.X, startPoint.Y));
            
            var dangerousWanderers = wanderers.Where(w => w.State == 1 || w.State == 0 && w.Time == 1).ToList();
            var minWandererDist = !dangerousWanderers.Any() ? 0 : dangerousWanderers.Min(w => GetManhattenDist(w.X, w.Y, startPoint.X, startPoint.Y));
            var minWanderersCount = GetWanderesCount(startPoint.X, startPoint.Y, myExplorerId, allExplorers, wanderers);
            var minVisibleSlashersCount = slashers.Count(s => IsVisblePoint(s, startPoint.X, startPoint.Y));

            Console.Error.WriteLine($"{startPoint.X} {startPoint.Y} {minExplorerDist} {minWanderersCount} {minVisibleSlashersCount} {minWandererDist}");

            foreach (Point neighbour in neighbours)
            {
                if (neighbour.Weight == BigValue)
                    continue;

                var explorerDist = allExplorers.Where(e => e.Id != myExplorerId).Min(e => GetManhattenDist(e.X, e.Y, neighbour.X, neighbour.Y));
                var wanderDist = !dangerousWanderers.Any() ?  0 : dangerousWanderers.Min(w => GetManhattenDist(w.X, w.Y, neighbour.X, neighbour.Y));
                var wanderersCount = GetWanderesCount(neighbour.X, neighbour.Y, myExplorerId, allExplorers, wanderers);
                var visibleSlashersCount =
                    slashers.Count(s => IsVisblePoint(s, neighbour.X, neighbour.Y));

                Console.Error.WriteLine($"{neighbour.X} {neighbour.Y} {explorerDist} {wanderersCount} {visibleSlashersCount} {wanderDist}");

                if (neighbour.Weight < minWeight)
                {
                    minWeight = neighbour.Weight;
                    minWeightPoint = neighbour;
                    minExplorerDist = explorerDist;
                    minWanderersCount = wanderersCount;
                    minVisibleSlashersCount = visibleSlashersCount;
                    minWandererDist = wanderDist;
                }
                else if (neighbour.Weight == minWeight)
                {
                    //TODO: дикие пляски с аурой
                    if (explorerDist < minExplorerDist && minExplorerDist >= AuraRange)
                    {
                        minWeight = neighbour.Weight;
                        minWeightPoint = neighbour;
                        minExplorerDist = explorerDist;
                        minWanderersCount = wanderersCount;
                        minVisibleSlashersCount = visibleSlashersCount;
                        minWandererDist = wanderDist;
                    }
                    else if (explorerDist <= minExplorerDist)
                    {
                        if (wanderersCount < minWanderersCount)
                        {
                            minWeight = neighbour.Weight;
                            minWeightPoint = neighbour;
                            minExplorerDist = explorerDist;
                            minWanderersCount = wanderersCount;
                            minVisibleSlashersCount = visibleSlashersCount;
                            minWandererDist = wanderDist;
                        }
                        else if (wanderersCount == minWanderersCount)
                        {
                            if (visibleSlashersCount < minVisibleSlashersCount)
                            {
                                minWeight = neighbour.Weight;
                                minWeightPoint = neighbour;
                                minExplorerDist = explorerDist;
                                minWanderersCount = wanderersCount;
                                minVisibleSlashersCount = visibleSlashersCount;
                                minWandererDist = wanderDist;
                            }
                            else if (visibleSlashersCount == minVisibleSlashersCount)
                            {
                                if (wanderDist > minWandererDist)
                                {
                                    minWeight = neighbour.Weight;
                                    minWeightPoint = neighbour;
                                    minExplorerDist = explorerDist;
                                    minWanderersCount = wanderersCount;
                                    minVisibleSlashersCount = visibleSlashersCount;
                                    minWandererDist = wanderDist;
                                }
                            }
                        }
                    }
                }
            }

            return minWeightPoint;
        }



        static CellPoint GetGoPoint(Explorer myExplorer, IList<Explorer> allExplorers, IList<Shelter> shelters, IList<Point> points)
        {
            var startPoint = points.Single(p => p.X == myExplorer.X && p.Y == myExplorer.Y);

            CellPoint nearestPoint = null;
            var minDist = int.MaxValue;
            foreach (var explorer in allExplorers)
            {
                if (explorer.Id == myExplorer.Id) continue;

                var finalPoint = points.Single(p => p.X == explorer.X && p.Y == explorer.Y);
                var path = Calculator.GetPath(startPoint, finalPoint, points);

                var dist = path.Sum(p => (p as Point).Weight);
                if (dist < minDist)
                {
                    nearestPoint = explorer;
                    minDist = dist;
                }
            }

            //TODO!!!
            //foreach (var shelter in shelters.Where(s => s.RemainigEnergy > 0))
            //{
            //    var finalPoint = points.Single(p => p.X == shelter.X && p.Y == shelter.Y);
            //    var path = Calculator.GetPath(startPoint, finalPoint, points);

            //    var dist = path.Sum(p => (p as Point).Weight);
            //    Console.Error.WriteLine($"{shelter.Id} {dist}");
            //    if (dist < minDist)
            //    {
            //        nearestPoint = shelter;
            //        minDist = dist;
            //    }
            //}

            return nearestPoint;
        }

        static IDictionary<Point, int> GetWandererDangerousPoints(CellPoint myExplorerPoint, IList<Wanderer> wanderers, IList<Point> points)
        {
            var wandererDangerousPoints = new Dictionary<Point, int>();
            foreach (var wanderer in wanderers.Where(w => w.State > 0 || w.State == 1 && w.Time == 1)) //пропускаем спящих
            {
                var wPoint = points.Single(p => p.X == wanderer.X && p.Y == wanderer.Y);

                //TODO: подумать о проходимости несоседних клеток

                //Клетка приведения
                //if (GetManhattenDist(myExplorerPoint.X, myExplorerPoint.Y, wPoint.X, wPoint.Y) == 1)
                //{
                    if (!wandererDangerousPoints.ContainsKey(wPoint)) wandererDangerousPoints.Add(wPoint, 0);
                    wandererDangerousPoints[wPoint]++;
                //}

                //Клетки, куда оно может пойти
                var wNeighbours = wPoint.GetNeighbors(points);
                foreach (Point neighbour in wNeighbours)
                {
                    if (neighbour.Weight == BigValue) continue;//стена
                    var dist = GetManhattenDist(myExplorerPoint.X, myExplorerPoint.Y, neighbour.X, neighbour.Y);
                    if (dist <= 1)
                    {
                        if (!wandererDangerousPoints.ContainsKey(neighbour)) wandererDangerousPoints.Add(neighbour, 0);
                        wandererDangerousPoints[neighbour]++;
                    }

                }
            }

            return wandererDangerousPoints;
        }

        static bool IsVisblePoint(CellPoint source, int destX, int destY)
        {
            if (source.X != destX && source.Y != destY) return false;
            if (source.X == destX)
            {
                var minY = Math.Min(source.Y, destY);
                var maxY = Math.Max(source.Y, destY);
                return Points.Where(p => p.X == source.X && p.Y > minY && p.Y < maxY).All(p => p.Weight < BigValue);
            }

            var minX = Math.Min(source.X, destX);
            var maxX = Math.Max(source.X, destX);
            return Points.Where(p => p.Y == source.Y && p.X > minX && p.X < maxX).All(p => p.Weight < BigValue);
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


        static IDictionary<Point, int> GetSlasherDangerousPoints(IList<Explorer> allExplorers, IList<Wanderer> slashers, IList<Point> points, Explorer myExplorer)
        {
            var res = new Dictionary<Point, int>();
            foreach (var slasher in slashers.Where(s => s.State == 3 || s.State == 2 && s.Time == 1 || s.State == 0 && s.Time == 1)) //TODO: State == 2
            {
                var slasherPoint = points.Single(p => p.X == slasher.X && p.Y == slasher.Y);

                if (slasher.State == 2 && slasher.Time == 1 || slasher.State == 0 && slasher.Time == 1)
                {
                    var isMyExplorerVisible = IsVisblePoint(slasher, myExplorer.X, myExplorer.Y);
                    if (isMyExplorerVisible)
                    {
                        Point n1 = null, n2 = null;
                        if (slasher.Y == myExplorer.Y)
                        {
                            n1 = points.Single(p => p.X == myExplorer.X && p.Y == myExplorer.Y - 1);
                            n2 = points.Single(p => p.X == myExplorer.X && p.Y == myExplorer.Y + 1);
                        }
                        else
                        {
                            n1 = points.Single(p => p.Y == myExplorer.Y && p.X == myExplorer.X - 1);
                            n2 = points.Single(p => p.Y == myExplorer.Y && p.X == myExplorer.X + 1);
                        }
                        var canIHide = n1.Weight < BigValue || n2.Weight < BigValue;
                        if (!canIHide)
                        {
                            var myExplorerePoint = points.Single(p => p.X == myExplorer.X && p.Y == myExplorer.Y);
                            if (!res.ContainsKey(myExplorerePoint)) res.Add(myExplorerePoint, 0);
                            res[myExplorerePoint]++;
                        }

                    }
                    continue;
                }

                //TODO: just after spawn
                var targetExplorer = allExplorers.SingleOrDefault(e => e.Id == slasher.TargetId);
                var isVisible = targetExplorer != null && IsVisblePoint(slasher, targetExplorer.X, targetExplorer.Y);
                var canHide = false;
                if (targetExplorer != null && isVisible)
                {
                    Point n1 = null, n2 = null;
                    if (slasher.Y == targetExplorer.Y)
                    {
                        n1 = points.Single(p => p.X == targetExplorer.X && p.Y == targetExplorer.Y - 1);
                        n2 = points.Single(p => p.X == targetExplorer.X && p.Y == targetExplorer.Y + 1);
                    }
                    else
                    {
                        n1 = points.Single(p => p.Y == targetExplorer.Y && p.X == targetExplorer.X - 1);
                        n2 = points.Single(p => p.Y == targetExplorer.Y && p.X == targetExplorer.X + 1);
                    }

                    canHide = n1.Weight < BigValue || n2.Weight < BigValue;
                }

                if (targetExplorer == null || slasher.TargetId == myExplorer.Id || !isVisible || canHide) //почемаем все точки на LOS опасными
                {
                    var currPoint = slasherPoint;
                    if (!res.ContainsKey(currPoint)) res.Add(currPoint, 0);
                    res[currPoint]++;

                    currPoint = slasherPoint;
                    while (true)
                    {
                        currPoint = points.SingleOrDefault(p => p.X == currPoint.X - 1 && p.Y == currPoint.Y && p.Weight < BigValue);
                        if (currPoint == null) break;
                        if (!res.ContainsKey(currPoint)) res.Add(currPoint, 0);
                        res[currPoint]++;
                    }
                    currPoint = slasherPoint;
                    while (true)
                    {
                        currPoint = points.SingleOrDefault(p => p.X == currPoint.X + 1 && p.Y == currPoint.Y && p.Weight < BigValue);
                        if (currPoint == null) break;
                        if (!res.ContainsKey(currPoint)) res.Add(currPoint, 0);
                        res[currPoint]++;
                    }

                    currPoint = slasherPoint;
                    while (true)
                    {
                        currPoint = points.SingleOrDefault(p => p.X == currPoint.X && p.Y == currPoint.Y - 1 && p.Weight < BigValue);
                        if (currPoint == null) break;
                        if (!res.ContainsKey(currPoint)) res.Add(currPoint, 0);
                        res[currPoint]++;
                    }
                    currPoint = slasherPoint;
                    while (true)
                    {
                        currPoint = points.SingleOrDefault(p => p.X == currPoint.X && p.Y == currPoint.Y + 1 && p.Weight < BigValue);
                        if (currPoint == null) break;
                        if (!res.ContainsKey(currPoint)) res.Add(currPoint, 0);
                        res[currPoint]++;
                    }

                    //if (!SlasherTargets.ContainsKey(slasher.Id)) continue;
                    //targetExplorer = allExplorers.SingleOrDefault(e => e.Id == SlasherTargets[slasher.Id]);
                    //if (targetExplorer == null) continue;

                    //var targetExplorerPoint = points.Single(p => p.X == targetExplorer.X && p.Y == targetExplorer.Y);
                    //var path = Calculator.GetPath(slasherPoint, targetExplorerPoint, points);//TODO: путь м.б. неверным

                    //if (path.Count == 1)
                    //{
                    //    if (!res.ContainsKey(slasherPoint)) res.Add(slasherPoint, 0);
                    //    res[slasherPoint]++;
                    //}
                    //else
                    //{
                    //    var index = 1;
                    //    var currPoint = path[index] as Point;

                    //    while (true)
                    //    {
                    //        var newIndex = index + 1;
                    //        if (newIndex > path.Count - 1) break;

                    //        var newPoint = path[newIndex] as Point;
                    //        if (newPoint.X != slasherPoint.X || newPoint.Y != slasherPoint.Y) break;

                    //        currPoint = newPoint;
                    //    }

                    //    if (!res.ContainsKey(currPoint)) res.Add(currPoint, 0);
                    //    res[currPoint]++;
                    //}
                    //continue;
                }

                
                //var targetPoint = points.Single(p => p.X == targetExplorer.X && p.Y == targetExplorer.Y);

                //var sPath = Calculator.GetPath(slasherPoint, targetPoint, points);
                //if (sPath.Count == 1)
                //{
                //    if (!res.ContainsKey(slasherPoint)) res.Add(slasherPoint, 1);
                //    continue;
                //}

                //Point nextStep = sPath[1] as Point;
               
                ////помечаем все точки на LoS слэшера
                //if (nextStep.X != slasher.X)
                //{
                //    var currPoint = slasherPoint;
                //    while (currPoint != null)
                //    {
                //        if (!res.ContainsKey(currPoint)) res.Add(currPoint, 0);
                //        res[currPoint]++;
                //        currPoint = points.SingleOrDefault(p => p.X == currPoint.X - 1 && p.Y == currPoint.Y && p.Weight < BigValue);
                //    }
                //    currPoint = slasherPoint;
                //    while (currPoint != null)
                //    {
                //        if (!res.ContainsKey(currPoint)) res.Add(currPoint, 0);
                //        res[currPoint]++;
                //        currPoint = points.SingleOrDefault(p => p.X == currPoint.X + 1 && p.Y == currPoint.Y && p.Weight < BigValue);
                //    }
                //}
                //else
                //{
                //    var currPoint = slasherPoint;
                //    while (currPoint != null)
                //    {
                //        if (!res.ContainsKey(currPoint)) res.Add(currPoint, 0);
                //        res[currPoint]++;
                //        currPoint = points.SingleOrDefault(p => p.X == currPoint.X && p.Y == currPoint.Y - 1 && p.Weight < BigValue);
                //    }
                //    currPoint = slasherPoint;
                //    while (currPoint != null)
                //    {
                //        if(!res.ContainsKey(currPoint)) res.Add(currPoint, 0);
                //        res[currPoint]++;
                //        currPoint = points.SingleOrDefault(p => p.X == currPoint.X && p.Y == currPoint.Y + 1 && p.Weight < BigValue);
                //    }
                //}


                //TODO!!!
                //var slasherTargetCellPoint = GetSlasherTargetPoint(slasher, allExplorers);
                //if (slasherTargetCellPoint != null)
                //{
                //    var slasherTargetPoint = points.Single(p =>
                //        p.X == slasherTargetCellPoint.X && p.Y == slasherTargetCellPoint.Y);
                //    if (!res.ContainsKey(slasherTargetPoint)) res.Add(slasherTargetPoint, 0);
                //    res[slasherTargetPoint]++;
                //}

                

                //if (res.ContainsKey(targetPoint))
                //    res[targetPoint]++;//текущую точку прыжка делаем самой опасной
                //else
                //{
                //    Explorer nearestLosExplorer = null;
                //    var minDist = int.MaxValue;
                //    foreach (var explorer in allExplorers.Where(e => e.X == slasher.X || e.Y == slasher.Y))
                //    {
                //        var dist = GetManhattenDist(explorer, slasher);
                //        if (dist < minDist)
                //        {
                //            minDist = dist;
                //            nearestLosExplorer = explorer;
                //        }
                //    }

                //    if (nearestLosExplorer != null)
                //    {
                //        var nearestLosExplorerPoint =
                //            points.Single(p => p.X == nearestLosExplorer.X && p.Y == nearestLosExplorer.Y);
                //        if (!res.ContainsKey(nearestLosExplorerPoint)) res.Add(nearestLosExplorerPoint, 0);
                //        res[nearestLosExplorerPoint]++;
                //    }

                //}
            }

            return res;
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
                    var afterLightWandererPoint = Points.Single(p => p.X == point2.X && p.Y == point2.Y);
                    var afterLightExplorerPoint = ExplorerPoints[explorer.Id];

                    var afterLightMyPath = Calculator.GetPath(afterLightWandererPoint, noLightMyPoint, Points);
                    var afterLightExplorerPath =
                        Calculator.GetPath(afterLightWandererPoint, afterLightExplorerPoint, Points);

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


        private static void UpdateSlasherTargets(IList<Wanderer> slashers)
        {
            foreach (var slasher in slashers)
            {
                if (!SlasherTargets.ContainsKey(slasher.Id))
                    SlasherTargets.Add(slasher.Id, slasher.TargetId);
                else
                    SlasherTargets[slasher.Id] = slasher.TargetId;
            }
        }




        #region static and constants values

        private static IList<Point> Points { get; set; }
        private static IDictionary<int, int> SlasherTargets = new Dictionary<int, int>();
        private static IDictionary<int, Point> ExplorerPoints = new Dictionary<int, Point>();
        private static IDictionary<int, Point> WandererPoints = new Dictionary<int, Point>();
        private static IDictionary<int, Point> SlasherPoints = new Dictionary<int, Point>();

        private static IDictionary<int, Dictionary<int, int>> WandererExplorerPathDistances = new Dictionary<int, Dictionary<int, int>>();

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

        private const int AuraRange = 2;

        private const int ShelterDefaultEnergy = 10;
        private const int MaxShelterEnergyLeak = 3;
        
        #endregion

    }
}
