using System;
using System.Collections.Generic;
using System.Linq;

namespace DeepSeekAnnealing
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("=== AUTOMATIC SPOT WELDING LINE ===");
            Console.WriteLine("Algorithm: Simulated Annealing\n");

            // Step 1: Initialize data
            var allPoints = LoadWeldingPoints();
            var robots = LoadRobots();
            var posts = LoadPosts(robots);
            var weldingGuns = LoadWeldingGuns();

            // Step 2: Assign gun types to robots
            AssignGunTypesToRobots(robots, weldingGuns);

            // Step 3: Display initial data
            DisplayInitialData(allPoints, robots, weldingGuns);

            // Step 4: Find optimal distribution using simulated annealing
            Console.WriteLine("\nSEARCHING FOR OPTIMAL DISTRIBUTION:");
            Console.WriteLine("Method: Simulated Annealing with Local Search\n");

            var saSolver = new SimulatedAnnealingSolver(allPoints, robots, posts, weldingGuns);
            var bestSolution = saSolver.FindOptimalDistribution();

            // Step 5: Apply the best distribution
            ApplyBestDistribution(allPoints, robots, bestSolution);

            // Step 6: Calculate timing parameters
            CalculateAllTimes(robots, weldingGuns, posts);

            // Step 7: Display results
            DisplayResults(allPoints, robots, posts, weldingGuns, bestSolution);

            Console.WriteLine("\n=== OPTIMAL DISTRIBUTION SEARCH COMPLETED ===");
        }

        // Data classes
        public class WeldingPoint
        {
            public string Name { get; set; }
            public int Segment { get; set; }
            public List<string> CompatibleGunTypes { get; set; }
            public string BelongsTo { get; set; }
            public double X { get; set; }
            public double Y { get; set; }
            public double Z { get; set; }
            public string Side { get; set; }
        }

        public class Robot
        {
            public string Name { get; set; }
            public string Side { get; set; }
            public string GunName { get; set; }
            public string GunType { get; set; }
            public string Object { get; set; }
            public double HomeX { get; set; }
            public double HomeY { get; set; }
            public double HomeZ { get; set; }
            public double Speed { get; set; }
            public List<WeldingPoint> AssignedPoints { get; set; } = new List<WeldingPoint>();
            public double CycleTime { get; set; }
            public double WearPercentage { get; set; }
        }

        public class Post
        {
            public int Number { get; set; }
            public List<Robot> LeftRobots { get; set; } = new List<Robot>();
            public List<Robot> RightRobots { get; set; } = new List<Robot>();
            public double PostCycleTime { get; set; }
            public double TransportTime { get; set; }
        }

        public class WeldingGun
        {
            public string Name { get; set; }
            public string Type { get; set; }
            public double WeldTimePerPoint { get; set; }
            public double Weight { get; set; }
            public double Wear { get; set; }
        }

        public class DistributionSolution
        {
            public Dictionary<string, List<string>> RobotAssignments { get; set; } = new Dictionary<string, List<string>>();
            public double Score { get; set; }
            public int IterationFound { get; set; }
            public double Temperature { get; set; }
            public string MoveType { get; set; }

            public DistributionSolution DeepCopy()
            {
                var copy = new DistributionSolution
                {
                    Score = this.Score,
                    IterationFound = this.IterationFound,
                    Temperature = this.Temperature,
                    MoveType = this.MoveType
                };

                copy.RobotAssignments = new Dictionary<string, List<string>>();
                foreach (var kvp in this.RobotAssignments)
                {
                    copy.RobotAssignments[kvp.Key] = new List<string>(kvp.Value);
                }

                return copy;
            }
        }


        // Simulated annealing solver class
        public class SimulatedAnnealingSolver
        {
            private List<WeldingPoint> allPoints;
            private List<Robot> robots;
            private List<Post> posts;
            private List<WeldingGun> weldingGuns;
            private Random random;
            private Dictionary<string, WeldingPoint> pointDict;

            public SimulatedAnnealingSolver(List<WeldingPoint> points, List<Robot> robots,
                                          List<Post> posts, List<WeldingGun> guns)
            {
                this.allPoints = points;
                this.robots = robots;
                this.posts = posts;
                this.weldingGuns = guns;
                this.random = new Random(DateTime.Now.Millisecond);
                this.pointDict = points.ToDictionary(p => p.Name);
            }

            public DistributionSolution FindOptimalDistribution()
            {
                Console.WriteLine("Starting simulated annealing algorithm...\n");

                // Algorithm parameters
                double initialTemperature = 1000.0;
                double finalTemperature = 0.1;
                double coolingRate = 0.95;
                int iterationsPerTemperature = 100;
                int maxIterationsWithoutImprovement = 50;

                // 1. Generate initial solution
                Console.WriteLine("Generating initial solution...");
                var currentSolution = GenerateInitialSolution();
                currentSolution.Score = CalculateSolutionScore(currentSolution);
                Console.WriteLine($"  Initial score: {currentSolution.Score:F2}");

                var bestSolution = currentSolution.DeepCopy();
                bestSolution.IterationFound = 0;

                // 2. Main simulated annealing loop
                double temperature = initialTemperature;
                int iteration = 0;
                int iterationsWithoutImprovement = 0;

                Console.WriteLine("\nStarting annealing process:");
                Console.WriteLine("Temp | Iteration | Current Score | Best Score | Accepted");
                Console.WriteLine(new string('-', 70));

                while (temperature > finalTemperature && iterationsWithoutImprovement < maxIterationsWithoutImprovement)
                {
                    bool acceptedInThisTemperature = false;

                    for (int i = 0; i < iterationsPerTemperature; i++)
                    {
                        iteration++;

                        // 3. Generate neighbor solution
                        var neighborSolution = GenerateNeighbor(currentSolution);

                        // 4. Local optimization of neighbor
                        neighborSolution = LocalOptimization(neighborSolution, 5);
                        neighborSolution.Score = CalculateSolutionScore(neighborSolution);

                        // 5. Calculate score difference
                        double delta = neighborSolution.Score - currentSolution.Score;

                        // 6. Decision criterion
                        bool accept = false;
                        string moveType = "";

                        if (delta < 0)
                        {
                            // Better solution - always accept
                            accept = true;
                            moveType = "IMPROVEMENT";
                        }
                        else if (temperature > 0)
                        {
                            // Worse solution - accept with probability exp(-Δ/T)
                            double acceptanceProbability = Math.Exp(-delta / temperature);
                            if (random.NextDouble() < acceptanceProbability)
                            {
                                accept = true;
                                moveType = $"ACCEPTED (p={acceptanceProbability:F3})";
                            }
                            else
                            {
                                moveType = "REJECTED";
                            }
                        }

                        // 7. Update current solution
                        if (accept)
                        {
                            currentSolution = neighborSolution;
                            acceptedInThisTemperature = true;

                            // 8. Update best solution
                            if (currentSolution.Score < bestSolution.Score)
                            {
                                bestSolution = currentSolution.DeepCopy();
                                bestSolution.IterationFound = iteration;
                                bestSolution.Temperature = temperature;
                                iterationsWithoutImprovement = 0;

                                Console.WriteLine($"{temperature,5:F1} | {iteration,8} | {currentSolution.Score,15:F2} | {bestSolution.Score,13:F2} | {moveType}");
                            }
                        }
                    }

                    // 9. Cooling
                    temperature *= coolingRate;

                    if (!acceptedInThisTemperature)
                        iterationsWithoutImprovement++;
                }

                // 10. Final local optimization of best solution
                Console.WriteLine("\nFinal local optimization of best solution...");
                bestSolution = IntensiveLocalOptimization(bestSolution, 50);
                bestSolution.Score = CalculateSolutionScore(bestSolution);

                Console.WriteLine($"\nAlgorithm completed in {iteration} iterations");
                Console.WriteLine($"Best solution found at iteration {bestSolution.IterationFound}");
                Console.WriteLine($"Final score: {bestSolution.Score:F2}");

                return bestSolution;
            }

            // 1. Generate initial solution
            private DistributionSolution GenerateInitialSolution()
            {
                var solution = new DistributionSolution();

                // Initialize assignments dictionary
                foreach (var robot in robots)
                {
                    solution.RobotAssignments[robot.Name] = new List<string>();
                }

                // Split points by sides
                var leftPoints = allPoints.Where(p => p.Side == "Left").ToList();
                var rightPoints = allPoints.Where(p => p.Side == "Right").ToList();

                // Group robots by sides
                var leftRobots = robots.Where(r => r.Side == "Left").ToList();
                var rightRobots = robots.Where(r => r.Side == "Right").ToList();

                // Generate initial solution using greedy algorithm
                GenerateGreedyInitialSolution(leftPoints, leftRobots, solution);
                GenerateGreedyInitialSolution(rightPoints, rightRobots, solution);

                return solution;
            }

            private void GenerateGreedyInitialSolution(List<WeldingPoint> points, List<Robot> robots, DistributionSolution solution)
            {
                // Group points by segments
                var segments = points.GroupBy(p => p.Segment)
                                    .ToDictionary(g => g.Key, g => g.ToList());

                // Sort segments by number of points (descending)
                var sortedSegments = segments.OrderByDescending(kvp => kvp.Value.Count).ToList();

                foreach (var segment in sortedSegments)
                {
                    var segmentPoints = segment.Value;
                    var segmentId = segment.Key;

                    // Find compatible robots for this segment
                    var compatibleRobots = robots.Where(r =>
                        segmentPoints[0].CompatibleGunTypes.Contains(r.GunType)).ToList();

                    if (compatibleRobots.Count == 0) continue;

                    // Sort robots by current load
                    compatibleRobots.Sort((a, b) =>
                        solution.RobotAssignments[a.Name].Count.CompareTo(
                        solution.RobotAssignments[b.Name].Count));

                    // Assign entire segment to one robot (if possible)
                    bool assignedAsWhole = false;
                    foreach (var robot in compatibleRobots)
                    {
                        // Check if robot can take the entire segment
                        if (solution.RobotAssignments[robot.Name].Count + segmentPoints.Count <= 15) // Maximum 15 points
                        {
                            foreach (var point in segmentPoints)
                            {
                                solution.RobotAssignments[robot.Name].Add(point.Name);
                            }
                            assignedAsWhole = true;
                            break;
                        }
                    }

                    // If couldn't assign entire segment, split it
                    if (!assignedAsWhole)
                    {
                        int pointsPerRobot = segmentPoints.Count / compatibleRobots.Count;
                        int remainder = segmentPoints.Count % compatibleRobots.Count;

                        int currentIndex = 0;
                        for (int i = 0; i < compatibleRobots.Count; i++)
                        {
                            int takeCount = pointsPerRobot + (i < remainder ? 1 : 0);
                            if (currentIndex + takeCount > segmentPoints.Count) break;

                            var robotPoints = segmentPoints.Skip(currentIndex).Take(takeCount);
                            foreach (var point in robotPoints)
                            {
                                solution.RobotAssignments[compatibleRobots[i].Name].Add(point.Name);
                            }
                            currentIndex += takeCount;
                        }
                    }
                }

                // Handle remaining points
                var allAssigned = solution.RobotAssignments.Values.SelectMany(x => x).Distinct().ToList();
                var unassignedPoints = points.Where(p => !allAssigned.Contains(p.Name)).ToList();

                foreach (var point in unassignedPoints)
                {
                    var compatibleRobots = robots.Where(r =>
                        point.CompatibleGunTypes.Contains(r.GunType)).ToList();

                    if (compatibleRobots.Count == 0) continue;

                    compatibleRobots.Sort((a, b) =>
                        solution.RobotAssignments[a.Name].Count.CompareTo(
                        solution.RobotAssignments[b.Name].Count));

                    solution.RobotAssignments[compatibleRobots[0].Name].Add(point.Name);
                }
            }

            // 2. Generate neighbor solution
            private DistributionSolution GenerateNeighbor(DistributionSolution current)
            {
                var neighbor = current.DeepCopy();

                // Choose mutation type
                int mutationType = random.Next(6);

                switch (mutationType)
                {
                    case 0: // Move single point
                        MoveSinglePoint(neighbor);
                        neighbor.MoveType = "Move point";
                        break;

                    case 1: // Swap two points between robots
                        SwapPointsBetweenRobots(neighbor);
                        neighbor.MoveType = "Swap points";
                        break;

                    case 2: // Redistribute segment
                        RedistributeSegment(neighbor);
                        neighbor.MoveType = "Segment redistribution";
                        break;

                    case 3: // Balance load
                        BalanceLoad(neighbor);
                        neighbor.MoveType = "Load balancing";
                        break;

                    case 4: // Randomize segment
                        RandomizeSegment(neighbor);
                        neighbor.MoveType = "Segment randomization";
                        break;

                    case 5: // Combined mutation
                        if (random.NextDouble() < 0.5)
                            MoveSinglePoint(neighbor);
                        else
                            SwapPointsBetweenRobots(neighbor);
                        neighbor.MoveType = "Combined mutation";
                        break;
                }

                // Clean duplicates
                CleanDuplicates(neighbor);

                return neighbor;
            }

            private void MoveSinglePoint(DistributionSolution solution)
            {
                // Choose random point
                var allAssigned = solution.RobotAssignments.Values.SelectMany(x => x).ToList();
                if (allAssigned.Count == 0) return;

                string pointName = allAssigned[random.Next(allAssigned.Count)];
                var point = pointDict[pointName];

                // Find current robot
                string currentRobotName = solution.RobotAssignments
                    .First(kvp => kvp.Value.Contains(pointName)).Key;

                // Find compatible robots (except current)
                var compatibleRobots = robots.Where(r =>
                    r.Name != currentRobotName &&
                    r.Side == point.Side &&
                    point.CompatibleGunTypes.Contains(r.GunType)).ToList();

                if (compatibleRobots.Count == 0) return;

                // Choose random compatible robot
                var newRobot = compatibleRobots[random.Next(compatibleRobots.Count)];

                // Move point
                solution.RobotAssignments[currentRobotName].Remove(pointName);
                solution.RobotAssignments[newRobot.Name].Add(pointName);
            }

            private void SwapPointsBetweenRobots(DistributionSolution solution)
            {
                // Choose two random robots
                var robotNames = solution.RobotAssignments.Keys.ToList();
                if (robotNames.Count < 2) return;

                string robot1Name = robotNames[random.Next(robotNames.Count)];
                string robot2Name = robotNames.Where(r => r != robot1Name)
                                             .ElementAtOrDefault(random.Next(robotNames.Count - 1));

                if (robot2Name == null ||
                    solution.RobotAssignments[robot1Name].Count == 0 ||
                    solution.RobotAssignments[robot2Name].Count == 0)
                    return;

                var robot1 = robots.First(r => r.Name == robot1Name);
                var robot2 = robots.First(r => r.Name == robot2Name);

                // Find points to swap
                var points1 = solution.RobotAssignments[robot1Name]
                    .Where(p => pointDict[p].CompatibleGunTypes.Contains(robot2.GunType) &&
                                pointDict[p].Side == robot2.Side)
                    .ToList();

                var points2 = solution.RobotAssignments[robot2Name]
                    .Where(p => pointDict[p].CompatibleGunTypes.Contains(robot1.GunType) &&
                                pointDict[p].Side == robot1.Side)
                    .ToList();

                if (points1.Count == 0 || points2.Count == 0) return;

                // Choose random points to swap
                string point1 = points1[random.Next(points1.Count)];
                string point2 = points2[random.Next(points2.Count)];

                // Perform swap
                solution.RobotAssignments[robot1Name].Remove(point1);
                solution.RobotAssignments[robot2Name].Remove(point2);
                solution.RobotAssignments[robot1Name].Add(point2);
                solution.RobotAssignments[robot2Name].Add(point1);
            }

            private void RedistributeSegment(DistributionSolution solution)
            {
                // Choose random segment
                var segments = allPoints.Select(p => p.Segment).Distinct().ToList();
                int segmentId = segments[random.Next(segments.Count)];

                var segmentPoints = allPoints.Where(p => p.Segment == segmentId).ToList();
                var segmentSide = segmentPoints[0].Side;
                var segmentGunTypes = segmentPoints[0].CompatibleGunTypes;

                // Find compatible robots
                var compatibleRobots = robots.Where(r =>
                    r.Side == segmentSide &&
                    segmentGunTypes.Contains(r.GunType)).ToList();

                if (compatibleRobots.Count == 0) return;

                // Remove all segment points from all robots
                var segmentPointNames = segmentPoints.Select(p => p.Name).ToList();
                foreach (var robot in robots)
                {
                    if (solution.RobotAssignments.ContainsKey(robot.Name))
                    {
                        solution.RobotAssignments[robot.Name] = solution.RobotAssignments[robot.Name]
                            .Where(p => !segmentPointNames.Contains(p))
                            .ToList();
                    }
                }

                // Redistribute segment
                if (random.NextDouble() < 0.7)
                {
                    // Assign entire segment to one robot
                    var selectedRobot = compatibleRobots[random.Next(compatibleRobots.Count)];
                    solution.RobotAssignments[selectedRobot.Name].AddRange(segmentPointNames);
                }
                else
                {
                    // Distribute among multiple robots
                    int pointsPerRobot = segmentPoints.Count / compatibleRobots.Count;
                    int remainder = segmentPoints.Count % compatibleRobots.Count;

                    int currentIndex = 0;
                    for (int i = 0; i < compatibleRobots.Count && currentIndex < segmentPoints.Count; i++)
                    {
                        int takeCount = pointsPerRobot + (i < remainder ? 1 : 0);
                        var robotPoints = segmentPoints.Skip(currentIndex).Take(takeCount);

                        foreach (var point in robotPoints)
                        {
                            solution.RobotAssignments[compatibleRobots[i].Name].Add(point.Name);
                        }

                        currentIndex += takeCount;
                    }
                }
            }

            private void BalanceLoad(DistributionSolution solution)
            {
                // Find robot with max and min load
                var robotLoads = robots.Select(r => new
                {
                    Robot = r,
                    Load = solution.RobotAssignments.ContainsKey(r.Name) ?
                           solution.RobotAssignments[r.Name].Count : 0
                }).ToList();

                var overloaded = robotLoads.OrderByDescending(r => r.Load).First();
                var underloaded = robotLoads.OrderBy(r => r.Load).First();

                if (overloaded.Robot == underloaded.Robot ||
                    overloaded.Load - underloaded.Load <= 2)
                    return;

                // Find point to move
                var pointsToMove = solution.RobotAssignments[overloaded.Robot.Name]
                    .Where(p =>
                    {
                        var point = pointDict[p];
                        return point.CompatibleGunTypes.Contains(underloaded.Robot.GunType) &&
                               point.Side == underloaded.Robot.Side;
                    })
                    .ToList();

                if (pointsToMove.Count == 0) return;

                // Move point
                string pointToMove = pointsToMove[random.Next(pointsToMove.Count)];
                solution.RobotAssignments[overloaded.Robot.Name].Remove(pointToMove);
                solution.RobotAssignments[underloaded.Robot.Name].Add(pointToMove);
            }

            private void RandomizeSegment(DistributionSolution solution)
            {
                // Choose random segment
                var segments = allPoints.Select(p => p.Segment).Distinct().ToList();
                int segmentId = segments[random.Next(segments.Count)];

                var segmentPoints = allPoints.Where(p => p.Segment == segmentId).ToList();
                var segmentSide = segmentPoints[0].Side;
                var segmentGunTypes = segmentPoints[0].CompatibleGunTypes;

                // Find compatible robots
                var compatibleRobots = robots.Where(r =>
                    r.Side == segmentSide &&
                    segmentGunTypes.Contains(r.GunType)).ToList();

                if (compatibleRobots.Count == 0) return;

                // Remove all segment points
                var segmentPointNames = segmentPoints.Select(p => p.Name).ToList();
                foreach (var robot in robots)
                {
                    if (solution.RobotAssignments.ContainsKey(robot.Name))
                    {
                        solution.RobotAssignments[robot.Name] = solution.RobotAssignments[robot.Name]
                            .Where(p => !segmentPointNames.Contains(p))
                            .ToList();
                    }
                }

                // Randomly distribute points
                foreach (var point in segmentPoints)
                {
                    var compatibleRobot = compatibleRobots[random.Next(compatibleRobots.Count)];
                    solution.RobotAssignments[compatibleRobot.Name].Add(point.Name);
                }
            }

            private void CleanDuplicates(DistributionSolution solution)
            {
                // Remove duplicate points
                var pointCount = new Dictionary<string, int>();

                foreach (var assignment in solution.RobotAssignments.Values)
                {
                    foreach (var pointName in assignment)
                    {
                        if (!pointCount.ContainsKey(pointName))
                            pointCount[pointName] = 0;
                        pointCount[pointName]++;
                    }
                }

                var duplicatePoints = pointCount.Where(kvp => kvp.Value > 1).Select(kvp => kvp.Key).ToList();

                foreach (var pointName in duplicatePoints)
                {
                    // Keep point only with one robot
                    var robotsWithPoint = robots.Where(r =>
                        solution.RobotAssignments[r.Name].Contains(pointName)).ToList();

                    // Choose robot with minimum load
                    robotsWithPoint.Sort((a, b) =>
                        solution.RobotAssignments[a.Name].Count.CompareTo(
                        solution.RobotAssignments[b.Name].Count));

                    var keeper = robotsWithPoint[0];

                    // Remove from others
                    foreach (var robot in robotsWithPoint.Skip(1))
                    {
                        solution.RobotAssignments[robot.Name].Remove(pointName);
                    }
                }

                // Add missing points
                var allAssigned = solution.RobotAssignments.Values.SelectMany(x => x).Distinct().ToList();
                var unassignedPoints = allPoints.Where(p => !allAssigned.Contains(p.Name)).ToList();

                foreach (var point in unassignedPoints)
                {
                    var compatibleRobots = robots.Where(r =>
                        r.Side == point.Side &&
                        point.CompatibleGunTypes.Contains(r.GunType)).ToList();

                    if (compatibleRobots.Count == 0) continue;

                    compatibleRobots.Sort((a, b) =>
                        solution.RobotAssignments[a.Name].Count.CompareTo(
                        solution.RobotAssignments[b.Name].Count));

                    solution.RobotAssignments[compatibleRobots[0].Name].Add(point.Name);
                }
            }

            // 3. Local optimization
            private DistributionSolution LocalOptimization(DistributionSolution solution, int maxIterations)
            {
                var optimized = solution.DeepCopy();
                double currentScore = CalculateSolutionScore(optimized);

                for (int i = 0; i < maxIterations; i++)
                {
                    // Try various local improvements
                    var improved = TryImproveByMovingPoints(optimized) ||
                                   TryImproveByConsolidatingSegments(optimized) ||
                                   TryImproveBySwappingPoints(optimized);

                    if (!improved) break;

                    double newScore = CalculateSolutionScore(optimized);
                    if (newScore >= currentScore) break;

                    currentScore = newScore;
                }

                return optimized;
            }

            private DistributionSolution IntensiveLocalOptimization(DistributionSolution solution, int iterations)
            {
                var best = solution.DeepCopy();
                double bestScore = CalculateSolutionScore(best);

                for (int i = 0; i < iterations; i++)
                {
                    var candidate = best.DeepCopy();

                    // Apply several mutations
                    for (int j = 0; j < 3; j++)
                    {
                        int mutationType = random.Next(3);
                        switch (mutationType)
                        {
                            case 0: MoveSinglePoint(candidate); break;
                            case 1: SwapPointsBetweenRobots(candidate); break;
                            case 2: BalanceLoad(candidate); break;
                        }
                    }

                    CleanDuplicates(candidate);

                    // Local optimization of candidate
                    candidate = LocalOptimization(candidate, 10);
                    double candidateScore = CalculateSolutionScore(candidate);

                    if (candidateScore < bestScore)
                    {
                        best = candidate;
                        bestScore = candidateScore;
                    }
                }

                return best;
            }

            private bool TryImproveByMovingPoints(DistributionSolution solution)
            {
                bool improved = false;

                // Analyze distribution by segments
                var segmentAnalysis = AnalyzeSegments(solution);

                // Look for segments split between robots
                foreach (var segment in segmentAnalysis.Where(s => s.RobotCount > 1))
                {
                    // Try to consolidate segment with one robot
                    if (TryConsolidateSegment(solution, segment.SegmentId))
                    {
                        improved = true;
                        break;
                    }
                }

                return improved;
            }

            private bool TryImproveByConsolidatingSegments(DistributionSolution solution)
            {
                bool improved = false;

                // For each robot check if we can improve its segments
                foreach (var robot in robots)
                {
                    if (!solution.RobotAssignments.ContainsKey(robot.Name)) continue;

                    var robotSegments = solution.RobotAssignments[robot.Name]
                        .Select(p => pointDict[p].Segment)
                        .Distinct()
                        .ToList();

                    // If robot has multiple segments, try to consolidate them
                    if (robotSegments.Count > 1)
                    {
                        foreach (var segmentId in robotSegments)
                        {
                            if (TryGiveWholeSegmentToRobot(solution, segmentId, robot))
                            {
                                improved = true;
                                break;
                            }
                        }
                    }

                    if (improved) break;
                }

                return improved;
            }

            private bool TryImproveBySwappingPoints(DistributionSolution solution)
            {
                // Look for mutually beneficial swaps
                var robotPairs = new List<(Robot, Robot)>();

                for (int i = 0; i < robots.Count; i++)
                {
                    for (int j = i + 1; j < robots.Count; j++)
                    {
                        if (robots[i].Side == robots[j].Side)
                        {
                            robotPairs.Add((robots[i], robots[j]));
                        }
                    }
                }

                foreach (var (robot1, robot2) in robotPairs)
                {
                    if (TryFindBeneficialSwap(solution, robot1, robot2))
                        return true;
                }

                return false;
            }

            private bool TryConsolidateSegment(DistributionSolution solution, int segmentId)
            {
                var segmentPoints = allPoints.Where(p => p.Segment == segmentId).ToList();
                var segmentSide = segmentPoints[0].Side;
                var segmentGunTypes = segmentPoints[0].CompatibleGunTypes;

                // Find all robots that have points of this segment
                var currentRobots = robots.Where(r =>
                    solution.RobotAssignments.ContainsKey(r.Name) &&
                    solution.RobotAssignments[r.Name].Any(p =>
                        pointDict[p].Segment == segmentId)).ToList();

                if (currentRobots.Count <= 1) return false;

                // Find best robot for this segment
                var compatibleRobots = robots.Where(r =>
                    r.Side == segmentSide &&
                    segmentGunTypes.Contains(r.GunType)).ToList();

                if (compatibleRobots.Count == 0) return false;

                // Choose robot with minimum load
                compatibleRobots.Sort((a, b) =>
                    solution.RobotAssignments[a.Name].Count.CompareTo(
                    solution.RobotAssignments[b.Name].Count));

                var bestRobot = compatibleRobots[0];

                // Move all segment points to best robot
                var segmentPointNames = segmentPoints.Select(p => p.Name).ToList();

                foreach (var robot in currentRobots)
                {
                    solution.RobotAssignments[robot.Name] = solution.RobotAssignments[robot.Name]
                        .Where(p => !segmentPointNames.Contains(p))
                        .ToList();
                }

                solution.RobotAssignments[bestRobot.Name].AddRange(segmentPointNames);

                return true;
            }

            private bool TryGiveWholeSegmentToRobot(DistributionSolution solution, int segmentId, Robot targetRobot)
            {
                var segmentPoints = allPoints.Where(p => p.Segment == segmentId).ToList();
                var segmentSide = segmentPoints[0].Side;
                var segmentGunTypes = segmentPoints[0].CompatibleGunTypes;

                // Check compatibility
                if (targetRobot.Side != segmentSide ||
                    !segmentGunTypes.Contains(targetRobot.GunType))
                    return false;

                // Find all segment points on other robots
                var otherRobotsPoints = new Dictionary<Robot, List<string>>();
                var segmentPointNames = segmentPoints.Select(p => p.Name).ToList();

                foreach (var robot in robots.Where(r => r != targetRobot))
                {
                    if (solution.RobotAssignments.ContainsKey(robot.Name))
                    {
                        var robotSegmentPoints = solution.RobotAssignments[robot.Name]
                            .Where(p => segmentPointNames.Contains(p))
                            .ToList();

                        if (robotSegmentPoints.Count > 0)
                            otherRobotsPoints[robot] = robotSegmentPoints;
                    }
                }

                if (otherRobotsPoints.Count == 0) return false;

                // Check if we can take these points
                int targetRobotCapacity = 15 - solution.RobotAssignments[targetRobot.Name].Count;
                int pointsToTake = otherRobotsPoints.Sum(kvp => kvp.Value.Count);

                if (pointsToTake > targetRobotCapacity) return false;

                // Take points from other robots
                foreach (var kvp in otherRobotsPoints)
                {
                    foreach (var pointName in kvp.Value)
                    {
                        solution.RobotAssignments[kvp.Key.Name].Remove(pointName);
                        solution.RobotAssignments[targetRobot.Name].Add(pointName);
                    }
                }

                return true;
            }

            private bool TryFindBeneficialSwap(DistributionSolution solution, Robot robot1, Robot robot2)
            {
                // Look for point pairs for mutually beneficial swap
                var points1 = solution.RobotAssignments[robot1.Name]
                    .Where(p => pointDict[p].CompatibleGunTypes.Contains(robot2.GunType))
                    .ToList();

                var points2 = solution.RobotAssignments[robot2.Name]
                    .Where(p => pointDict[p].CompatibleGunTypes.Contains(robot1.GunType))
                    .ToList();

                if (points1.Count == 0 || points2.Count == 0) return false;

                // Try different combinations
                for (int i = 0; i < Math.Min(5, points1.Count); i++)
                {
                    for (int j = 0; j < Math.Min(5, points2.Count); j++)
                    {
                        string point1 = points1[i];
                        string point2 = points2[j];

                        // Save current state
                        var backup1 = new List<string>(solution.RobotAssignments[robot1.Name]);
                        var backup2 = new List<string>(solution.RobotAssignments[robot2.Name]);

                        // Try swap
                        solution.RobotAssignments[robot1.Name].Remove(point1);
                        solution.RobotAssignments[robot2.Name].Remove(point2);
                        solution.RobotAssignments[robot1.Name].Add(point2);
                        solution.RobotAssignments[robot2.Name].Add(point1);

                        double newScore = CalculateSolutionScore(solution);
                        double oldScore = CalculateSolutionScoreForBackup(backup1, backup2, robot1, robot2);

                        if (newScore < oldScore)
                        {
                            return true; // Improvement found
                        }
                        else
                        {
                            // Revert changes
                            solution.RobotAssignments[robot1.Name] = backup1;
                            solution.RobotAssignments[robot2.Name] = backup2;
                        }
                    }
                }

                return false;
            }

            private List<SegmentAnalysis> AnalyzeSegments(DistributionSolution solution)
            {
                var analysis = new List<SegmentAnalysis>();

                var segments = allPoints.Select(p => p.Segment).Distinct().ToList();

                foreach (var segmentId in segments)
                {
                    var segmentPoints = allPoints.Where(p => p.Segment == segmentId).ToList();

                    // Find robots that have points of this segment
                    var robotsWithSegment = new List<Robot>();

                    foreach (var robot in robots)
                    {
                        if (solution.RobotAssignments.ContainsKey(robot.Name))
                        {
                            var hasPoints = solution.RobotAssignments[robot.Name]
                                .Any(p => pointDict[p].Segment == segmentId);

                            if (hasPoints) robotsWithSegment.Add(robot);
                        }
                    }

                    analysis.Add(new SegmentAnalysis
                    {
                        SegmentId = segmentId,
                        TotalPoints = segmentPoints.Count,
                        RobotCount = robotsWithSegment.Count,
                        Robots = robotsWithSegment
                    });
                }

                return analysis;
            }

            private class SegmentAnalysis
            {
                public int SegmentId { get; set; }
                public int TotalPoints { get; set; }
                public int RobotCount { get; set; }
                public List<Robot> Robots { get; set; }
            }

            // 4. Solution scoring function
            public double CalculateSolutionScore(DistributionSolution solution)
            {
                double score = 0;

                // 1. Penalty for unassigned points
                var assigned = solution.RobotAssignments.Values.SelectMany(x => x).Distinct().Count();
                score += (allPoints.Count - assigned) * 1000;

                // 2. Penalty for duplicate points
                var allAssigned = solution.RobotAssignments.Values.SelectMany(x => x).ToList();
                var duplicateCount = allAssigned.Count - allAssigned.Distinct().Count();
                score += duplicateCount * 500;

                // 3. Penalty for gun type incompatibility
                foreach (var robot in robots)
                {
                    if (solution.RobotAssignments.ContainsKey(robot.Name))
                    {
                        foreach (var pointName in solution.RobotAssignments[robot.Name])
                        {
                            var point = pointDict[pointName];
                            if (!point.CompatibleGunTypes.Contains(robot.GunType))
                                score += 300;
                        }
                    }
                }

                // 4. Penalty for working on wrong side
                foreach (var robot in robots)
                {
                    if (solution.RobotAssignments.ContainsKey(robot.Name))
                    {
                        foreach (var pointName in solution.RobotAssignments[robot.Name])
                        {
                            var point = pointDict[pointName];
                            if (point.Side != robot.Side)
                                score += 1000;
                        }
                    }
                }

                // 5. Penalty for load imbalance between robots
                var robotLoads = robots.Select(r =>
                    solution.RobotAssignments.ContainsKey(r.Name) ?
                    solution.RobotAssignments[r.Name].Count : 0).ToList();

                if (robotLoads.Count > 0)
                {
                    double avgLoad = robotLoads.Average();
                    double imbalance = robotLoads.Sum(load => Math.Abs(load - avgLoad));
                    score += imbalance * 10;
                }

                // 6. Penalty for split segments
                var segmentAnalysis = AnalyzeSegments(solution);
                foreach (var segment in segmentAnalysis)
                {
                    if (segment.RobotCount > 1)
                    {
                        // Penalty increases with number of robots
                        score += (segment.RobotCount - 1) * 100;
                    }
                }

                // 7. Bonus for whole segments
                int wholeSegments = 0;
                foreach (var segment in segmentAnalysis)
                {
                    if (segment.RobotCount == 1)
                    {
                        var robot = segment.Robots[0];
                        int robotPointsInSegment = solution.RobotAssignments[robot.Name]
                            .Count(p => pointDict[p].Segment == segment.SegmentId);

                        if (robotPointsInSegment == segment.TotalPoints)
                        {
                            wholeSegments++;
                            score -= 150; // Big bonus for whole segment
                        }
                    }
                }

                // 8. Bonus for complete distribution
                if (assigned == allPoints.Count)
                {
                    score -= 2000;
                }

                // 9. Bonus for even distribution across posts
                var postLoads = new Dictionary<string, int>
                {
                    ["10"] = (solution.RobotAssignments.ContainsKey("R1") ? solution.RobotAssignments["R1"].Count : 0) +
                            (solution.RobotAssignments.ContainsKey("R2") ? solution.RobotAssignments["R2"].Count : 0),
                    ["20"] = (solution.RobotAssignments.ContainsKey("R3") ? solution.RobotAssignments["R3"].Count : 0) +
                            (solution.RobotAssignments.ContainsKey("R4") ? solution.RobotAssignments["R4"].Count : 0),
                    ["30"] = (solution.RobotAssignments.ContainsKey("R5") ? solution.RobotAssignments["R5"].Count : 0) +
                            (solution.RobotAssignments.ContainsKey("R6") ? solution.RobotAssignments["R6"].Count : 0),
                    ["40"] = (solution.RobotAssignments.ContainsKey("R7") ? solution.RobotAssignments["R7"].Count : 0) +
                            (solution.RobotAssignments.ContainsKey("R8") ? solution.RobotAssignments["R8"].Count : 0)
                };

                if (postLoads.Values.All(v => v > 0))
                {
                    double avgPost = postLoads.Values.Average();
                    double postImbalance = postLoads.Values.Sum(v => Math.Abs(v - avgPost));
                    score += postImbalance * 5;

                    // Bonus for perfect balancing
                    if (postImbalance == 0)
                        score -= 500;
                }

                // 10. Penalty for robot overload
                foreach (var load in robotLoads)
                {
                    if (load > 15) // Maximum 15 points per robot
                        score += (load - 15) * 100;
                }

                return score;
            }

            private double CalculateSolutionScoreForBackup(List<string> robot1Points, List<string> robot2Points,
                                                          Robot robot1, Robot robot2)
            {
                // Helper function for calculating score when checking swaps
                var tempSolution = new DistributionSolution();

                // Copy all assignments except the two robots being checked
                foreach (var robot in robots)
                {
                    if (robot.Name == robot1.Name)
                        tempSolution.RobotAssignments[robot.Name] = new List<string>(robot1Points);
                    else if (robot.Name == robot2.Name)
                        tempSolution.RobotAssignments[robot.Name] = new List<string>(robot2Points);
                    else
                        tempSolution.RobotAssignments[robot.Name] = new List<string>();
                }

                return CalculateSolutionScore(tempSolution);
            }
        }


        // Data loading (other methods same as in previous program)
        static List<WeldingPoint> LoadWeldingPoints()
        {
            var points = new List<WeldingPoint>();

            // Left side
            string[] ls1Points = { "pLS1.1", "pLS1.2", "pLS1.3", "pLS1.4", "pLS1.5", "pLS1.6", "pLS1.7", "pLS1.8", "pLS1.9", "pLS1.10" };
            foreach (var point in ls1Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 1,
                    CompatibleGunTypes = new List<string> { "GType_1" },
                    Side = "Left"
                });

            string[] ls2Points = { "pLS2.1", "pLS2.2", "pLS2.3", "pLS2.4", "pLS2.5" };
            foreach (var point in ls2Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 2,
                    CompatibleGunTypes = new List<string> { "GType_1", "GType_2" },
                    Side = "Left"
                });

            string[] ls3Points = { "pLS3.1", "pLS3.2", "pLS3.3", "pLS3.4", "pLS3.5" };
            foreach (var point in ls3Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 3,
                    CompatibleGunTypes = new List<string> { "GType_1", "GType_2" },
                    Side = "Left"
                });

            string[] ls4Points = { "pLS4.1", "pLS4.2", "pLS4.3", "pLS4.4", "pLS4.5", "pLS4.6", "pLS4.7", "pLS4.8", "pLS4.9", "pLS4.10" };
            foreach (var point in ls4Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 4,
                    CompatibleGunTypes = new List<string> { "GType_3" },
                    Side = "Left"
                });

            string[] ls5Points = { "pLS5.1", "pLS5.2", "pLS5.3", "pLS5.4", "pLS5.5", "pLS5.6", "pLS5.7", "pLS5.8", "pLS5.9", "pLS5.10" };
            foreach (var point in ls5Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 5,
                    CompatibleGunTypes = new List<string> { "GType_3" },
                    Side = "Left"
                });

            // Right side
            string[] rs6Points = { "pRS6.1", "pRS6.2", "pRS6.3", "pRS6.4", "pRS6.5", "pRS6.6", "pRS6.7", "pRS6.8", "pRS6.9", "pRS6.10" };
            foreach (var point in rs6Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 6,
                    CompatibleGunTypes = new List<string> { "GType_1" },
                    Side = "Right"
                });

            string[] rs7Points = { "pRS7.1", "pRS7.2", "pRS7.3", "pRS7.4", "pRS7.5" };
            foreach (var point in rs7Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 7,
                    CompatibleGunTypes = new List<string> { "GType_1", "GType_2" },
                    Side = "Right"
                });

            string[] rs8Points = { "pRS8.1", "pRS8.2", "pRS8.3", "pRS8.4", "pRS8.5" };
            foreach (var point in rs8Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 8,
                    CompatibleGunTypes = new List<string> { "GType_1", "GType_2" },
                    Side = "Right"
                });

            string[] rs9Points = { "pRS9.1", "pRS9.2", "pRS9.3", "pRS9.4", "pRS9.5", "pRS9.6", "pRS9.7", "pRS9.8", "pRS9.9", "pRS9.10" };
            foreach (var point in rs9Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 9,
                    CompatibleGunTypes = new List<string> { "GType_3" },
                    Side = "Right"
                });

            string[] rs10Points = { "pRS10.1", "pRS10.2", "pRS10.3", "pRS10.4", "pRS10.5", "pRS10.6", "pRS10.7", "pRS10.8", "pRS10.9", "pRS10.10" };
            foreach (var point in rs10Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 10,
                    CompatibleGunTypes = new List<string> { "GType_3" },
                    Side = "Right"
                });

            return points;
        }

        static List<Robot> LoadRobots()
        {
            return new List<Robot>
            {
                new Robot { Name = "R1", Side = "Left", GunName = "Gun1", Speed = 3000 },
                new Robot { Name = "R3", Side = "Left", GunName = "Gun2", Speed = 3000 },
                new Robot { Name = "R5", Side = "Left", GunName = "Gun3", Speed = 3000 },
                new Robot { Name = "R7", Side = "Left", GunName = "Gun4", Speed = 3000 },
                new Robot { Name = "R2", Side = "Right", GunName = "Gun5", Speed = 3000 },
                new Robot { Name = "R4", Side = "Right", GunName = "Gun6", Speed = 3000 },
                new Robot { Name = "R6", Side = "Right", GunName = "Gun7", Speed = 3000 },
                new Robot { Name = "R8", Side = "Right", GunName = "Gun8", Speed = 3000 }
            };
        }

        static List<Post> LoadPosts(List<Robot> robots)
        {
            var posts = new List<Post>();
            var robotDict = robots.ToDictionary(r => r.Name);

            posts.Add(new Post { Number = 10, TransportTime = 8 });
            posts.Add(new Post { Number = 20, TransportTime = 8 });
            posts.Add(new Post { Number = 30, TransportTime = 8 });
            posts.Add(new Post { Number = 40, TransportTime = 8 });

            foreach (var post in posts)
            {
                int postNum = post.Number;
                if (postNum == 10)
                {
                    post.LeftRobots.Add(robotDict["R1"]);
                    post.RightRobots.Add(robotDict["R2"]);
                }
                else if (postNum == 20)
                {
                    post.LeftRobots.Add(robotDict["R3"]);
                    post.RightRobots.Add(robotDict["R4"]);
                }
                else if (postNum == 30)
                {
                    post.LeftRobots.Add(robotDict["R5"]);
                    post.RightRobots.Add(robotDict["R6"]);
                }
                else if (postNum == 40)
                {
                    post.LeftRobots.Add(robotDict["R7"]);
                    post.RightRobots.Add(robotDict["R8"]);
                }
            }

            return posts;
        }

        static List<WeldingGun> LoadWeldingGuns()
        {
            return new List<WeldingGun>
            {
                new WeldingGun { Name = "Gun1", Type = "GType_1", WeldTimePerPoint = 2, Wear = 0 },
                new WeldingGun { Name = "Gun2", Type = "GType_2", WeldTimePerPoint = 2, Wear = 10 },
                new WeldingGun { Name = "Gun3", Type = "GType_3", WeldTimePerPoint = 2, Wear = 40 },
                new WeldingGun { Name = "Gun4", Type = "GType_3", WeldTimePerPoint = 2, Wear = 25 },
                new WeldingGun { Name = "Gun5", Type = "GType_1", WeldTimePerPoint = 2, Wear = 10 },
                new WeldingGun { Name = "Gun6", Type = "GType_2", WeldTimePerPoint = 2, Wear = 5 },
                new WeldingGun { Name = "Gun7", Type = "GType_3", WeldTimePerPoint = 2, Wear = 15 },
                new WeldingGun { Name = "Gun8", Type = "GType_3", WeldTimePerPoint = 2, Wear = 0 }
            };
        }

        static void AssignGunTypesToRobots(List<Robot> robots, List<WeldingGun> guns)
        {
            var gunDict = guns.ToDictionary(g => g.Name);
            foreach (var robot in robots)
            {
                if (gunDict.TryGetValue(robot.GunName, out var gun))
                {
                    robot.GunType = gun.Type;
                    robot.WearPercentage = gun.Wear;
                }
            }
        }

        static void DisplayInitialData(List<WeldingPoint> points, List<Robot> robots, List<WeldingGun> guns)
        {
            Console.WriteLine("INITIAL DATA:");
            Console.WriteLine("=============\n");

            Console.WriteLine($"Total welding points: {points.Count}");
            Console.WriteLine($"Left side: {points.Count(p => p.Side == "Left")} points");
            Console.WriteLine($"Right side: {points.Count(p => p.Side == "Right")} points");

            Console.WriteLine("\nDistribution by segments:");
            var segments = points.GroupBy(p => p.Segment)
                                .OrderBy(g => g.Key);

            foreach (var seg in segments)
            {
                Console.WriteLine($"  Segment {seg.Key}: {seg.Count()} points, Types: {string.Join("/", seg.First().CompatibleGunTypes)}");
            }

            Console.WriteLine("\nRobots:");
            foreach (var robot in robots.OrderBy(r => r.Name))
            {
                Console.WriteLine($"  {robot.Name}: {robot.Side} side, Guns: {robot.GunName} ({robot.GunType})");
            }

            Console.WriteLine("\n" + new string('=', 60));
        }

        static void ApplyBestDistribution(List<WeldingPoint> allPoints, List<Robot> robots, DistributionSolution solution)
        {
            Console.WriteLine("\nApplying found distribution...");

            foreach (var robot in robots)
            {
                robot.AssignedPoints.Clear();
            }

            var pointDict = allPoints.ToDictionary(p => p.Name);

            foreach (var kvp in solution.RobotAssignments)
            {
                var robot = robots.FirstOrDefault(r => r.Name == kvp.Key);
                if (robot != null)
                {
                    foreach (var pointName in kvp.Value)
                    {
                        if (pointDict.TryGetValue(pointName, out var point))
                        {
                            robot.AssignedPoints.Add(point);
                        }
                    }
                }
            }
        }

        static void CalculateAllTimes(List<Robot> robots, List<WeldingGun> guns, List<Post> posts)
        {
            var gunDict = guns.ToDictionary(g => g.Name);

            foreach (var robot in robots)
            {
                if (robot.AssignedPoints.Count == 0)
                {
                    robot.CycleTime = 0;
                    continue;
                }

                double weldTimePerPoint = 2.0;
                if (gunDict.TryGetValue(robot.GunName, out var gun))
                    weldTimePerPoint = gun.WeldTimePerPoint;

                // Simplified movement time calculation
                double movementTime = CalculateRobotMovementTime(robot);
                double weldingTime = robot.AssignedPoints.Count * weldTimePerPoint;
                robot.CycleTime = movementTime + weldingTime;
            }

            foreach (var post in posts)
            {
                double maxRobotTime = 0;
                var allPostRobots = post.LeftRobots.Concat(post.RightRobots);

                foreach (var robot in allPostRobots)
                    if (robot.CycleTime > maxRobotTime)
                        maxRobotTime = robot.CycleTime;

                post.PostCycleTime = maxRobotTime + post.TransportTime;
            }
        }

        static double CalculateRobotMovementTime(Robot robot)
        {
            if (robot.AssignedPoints.Count == 0) return 0;

            // More realistic movement time calculation
            double baseTime = 2.0;
            double timeBetweenPoints = 0.3;
            double returnTime = 1.0;

            return baseTime + (robot.AssignedPoints.Count - 1) * timeBetweenPoints + returnTime;
        }

        static void DisplayResults(List<WeldingPoint> allPoints, List<Robot> robots,
                                  List<Post> posts, List<WeldingGun> guns, DistributionSolution solution)
        {
            Console.WriteLine("\n" + new string('=', 70));
            Console.WriteLine("DISTRIBUTION RESULTS (Simulated Annealing)");
            Console.WriteLine(new string('=', 70) + "\n");

            // Table 1
            Console.WriteLine("TABLE 1: Distribution of welding points by robots");
            Console.WriteLine(new string('=', 100));
            Console.WriteLine("Post | Side    | Robot | Part Segments | Welding Points | Count | Robot Time(sec)");
            Console.WriteLine(new string('-', 100));

            foreach (var post in posts.OrderBy(p => p.Number))
            {
                var allPostRobots = post.LeftRobots.Concat(post.RightRobots).OrderBy(r => r.Name);

                foreach (var robot in allPostRobots)
                {
                    var segments = robot.AssignedPoints
                        .Select(p => p.Segment)
                        .Distinct()
                        .OrderBy(s => s)
                        .ToList();

                    string segmentsStr = string.Join(",", segments);

                    var pointNames = robot.AssignedPoints
                        .OrderBy(p => p.Name)
                        .Select(p => p.Name)
                        .ToList();

                    string pointsStr;
                    if (pointNames.Count <= 3)
                        pointsStr = string.Join(", ", pointNames);
                    else
                        pointsStr = $"{pointNames.First()},...,{pointNames.Last()}";

                    Console.WriteLine($"{post.Number,-4} | {robot.Side,-8} | {robot.Name,-5} | {segmentsStr,-16} | {pointsStr,-25} | {robot.AssignedPoints.Count,6} | {robot.CycleTime,17:F2}");
                }
            }

            Console.WriteLine(new string('-', 100));

            // Table 2
            Console.WriteLine("\nTABLE 2: Post summary");
            Console.WriteLine(new string('=', 50));
            Console.WriteLine("Post | Point Count | Post Cycle Time(sec)");
            Console.WriteLine(new string('-', 50));

            double totalLineTime = 0;
            foreach (var post in posts.OrderBy(p => p.Number))
            {
                int totalPoints = post.LeftRobots.Sum(r => r.AssignedPoints.Count) +
                                 post.RightRobots.Sum(r => r.AssignedPoints.Count);

                Console.WriteLine($"{post.Number,-4} | {totalPoints,13} | {post.PostCycleTime,24:F2}");
                totalLineTime += post.PostCycleTime;
            }

            Console.WriteLine(new string('-', 50));
            Console.WriteLine($"Total points: {allPoints.Count}");
            Console.WriteLine($"Maximum post time: {posts.Max(p => p.PostCycleTime):F2} sec");
            Console.WriteLine($"Total line time: {totalLineTime:F2} sec");

            // Table 3
            Console.WriteLine("\nTABLE 3: Welding gun assignment");
            Console.WriteLine(new string('=', 70));
            Console.WriteLine("Robot | Gun Type   | Point Count | Cycle Time(sec) | Segments");
            Console.WriteLine(new string('-', 70));

            foreach (var robot in robots.OrderBy(r => r.Name))
            {
                var segments = robot.AssignedPoints
                    .Select(p => p.Segment)
                    .Distinct()
                    .OrderBy(s => s)
                    .ToList();

                string segmentsStr = string.Join(",", segments);

                Console.WriteLine($"{robot.Name,-5} | {robot.GunType,-11} | {robot.AssignedPoints.Count,12} | {robot.CycleTime,17:F2} | {segmentsStr}");
            }

            Console.WriteLine(new string('-', 70));

            // Solution information
            Console.WriteLine("\nSOLUTION INFORMATION:");
            Console.WriteLine(new string('=', 30));
            Console.WriteLine($"Solution score: {solution.Score:F2}");
            Console.WriteLine($"Found at iteration: {solution.IterationFound}");
            Console.WriteLine($"Temperature when found: {solution.Temperature:F2}");

            // Validation checks
            Console.WriteLine("\nCORRECTNESS CHECK:");
            Console.WriteLine(new string('=', 30));
            PerformValidationChecks(allPoints, robots, posts);
        }

        static void PerformValidationChecks(List<WeldingPoint> allPoints, List<Robot> robots, List<Post> posts)
        {
            bool allValid = true;

            // 1. All points are distributed
            var allAssignedPoints = robots.SelectMany(r => r.AssignedPoints).ToList();
            var uniqueAssigned = allAssignedPoints.Select(p => p.Name).Distinct().ToList();

            if (uniqueAssigned.Count != allPoints.Count)
            {
                Console.WriteLine($"✗ {uniqueAssigned.Count} out of {allPoints.Count} points distributed");
                allValid = false;
            }
            else
            {
                Console.WriteLine($"✓ All {allPoints.Count} points distributed");
            }

            // 2. Each point assigned to only one robot
            if (allAssignedPoints.Count != uniqueAssigned.Count)
            {
                Console.WriteLine("✗ Some points assigned to more than one robot");
                allValid = false;
            }
            else
            {
                Console.WriteLine("✓ Each point assigned to exactly one robot");
            }

            // 3. All robots are utilized
            var inactiveRobots = robots.Where(r => r.AssignedPoints.Count == 0).ToList();
            if (inactiveRobots.Any())
            {
                Console.WriteLine($"✗ Robots not utilized: {string.Join(", ", inactiveRobots.Select(r => r.Name))}");
                allValid = false;
            }
            else
            {
                Console.WriteLine("✓ All robots utilized");
            }

            // 4. Robots work on their own sides
            bool sideValid = true;
            foreach (var robot in robots)
            {
                var wrongSidePoints = robot.AssignedPoints.Where(p => p.Side != robot.Side).ToList();
                if (wrongSidePoints.Any())
                {
                    Console.WriteLine($"✗ Robot {robot.Name} ({robot.Side} side) received points from other side");
                    sideValid = false;
                    allValid = false;
                }
            }
            if (sideValid) Console.WriteLine("✓ Robots work only on their own sides");

            // 5. Gun type compatibility
            bool compatibilityValid = true;
            foreach (var robot in robots)
            {
                var incompatiblePoints = robot.AssignedPoints
                    .Where(p => !p.CompatibleGunTypes.Contains(robot.GunType))
                    .ToList();

                if (incompatiblePoints.Any())
                {
                    Console.WriteLine($"✗ Robot {robot.Name} ({robot.GunType}) received incompatible points");
                    compatibilityValid = false;
                    allValid = false;
                }
            }
            if (compatibilityValid) Console.WriteLine("✓ All points compatible with robot gun types");

            // 6. Load balancing between posts
            var postTotals = posts.Select(p =>
                p.LeftRobots.Sum(r => r.AssignedPoints.Count) +
                p.RightRobots.Sum(r => r.AssignedPoints.Count)).ToList();

            double avgPoints = postTotals.Average();
            double imbalance = postTotals.Sum(p => Math.Abs(p - avgPoints));

            if (imbalance > 5)
                Console.WriteLine($"⚠ Uneven point distribution between posts (difference: {imbalance:F1} points)");
            else
                Console.WriteLine("✓ Load evenly distributed between posts");

            // 7. Time balancing
            var postTimes = posts.Select(p => p.PostCycleTime).ToList();
            double maxTime = postTimes.Max();
            double minTime = postTimes.Min();
            double timeImbalance = maxTime - minTime;

            if (timeImbalance > 5.0)
                Console.WriteLine($"⚠ Uneven post working time (difference: {timeImbalance:F2} sec)");
            else
                Console.WriteLine("✓ Post working time balanced");

            if (allValid)
                Console.WriteLine("\n✓ DISTRIBUTION IS CORRECT AND OPTIMAL!");
            else
                Console.WriteLine("\n✗ DISTRIBUTION HAS ERRORS!");
        }
    }
}