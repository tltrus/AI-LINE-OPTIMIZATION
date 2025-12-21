using System;
using System.Collections.Generic;
using System.Linq;

namespace DeepSeekGEN
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("=== AUTOMATIC SPOT WELDING LINE ===");
            Console.WriteLine("Objective: Search for optimal welding point distribution\n");

            // Step 1: Initialize data
            var allPoints = LoadWeldingPoints();
            var robots = LoadRobots();
            var posts = LoadPosts(robots);
            var weldingGuns = LoadWeldingGuns();

            // Step 2: Assign gun types to robots
            AssignGunTypesToRobots(robots, weldingGuns);

            // Step 3: Display initial data
            DisplayInitialData(allPoints, robots, weldingGuns);

            // Step 4: Search for optimal distribution
            Console.WriteLine("\nSEARCHING FOR OPTIMAL DISTRIBUTION:");
            Console.WriteLine("===================================\n");

            var bestDistribution = FindOptimalDistribution(allPoints, robots, posts, weldingGuns);

            // Step 5: Apply the best distribution
            ApplyBestDistribution(allPoints, robots, bestDistribution);

            // Step 6: Calculate timing parameters
            CalculateAllTimes(robots, weldingGuns, posts);

            // Step 7: Display results
            DisplayResults(allPoints, robots, posts, weldingGuns, bestDistribution);

            Console.WriteLine("\n=== OPTIMAL DISTRIBUTION SEARCH COMPLETED ===");
        }

        // Data classes
        class WeldingPoint
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

        class Robot
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

        class Post
        {
            public int Number { get; set; }
            public List<Robot> LeftRobots { get; set; } = new List<Robot>();
            public List<Robot> RightRobots { get; set; } = new List<Robot>();
            public double PostCycleTime { get; set; }
            public double TransportTime { get; set; }
        }

        class WeldingGun
        {
            public string Name { get; set; }
            public string Type { get; set; }
            public double WeldTimePerPoint { get; set; }
            public double Weight { get; set; }
            public double Wear { get; set; }
        }

        class Distribution
        {
            public Dictionary<string, List<string>> RobotAssignments { get; set; } = new Dictionary<string, List<string>>();
            public double Score { get; set; }
            public string Strategy { get; set; }
            public string Description { get; set; }
        }

        // 1. Load data with real points
        static List<WeldingPoint> LoadWeldingPoints()
        {
            var points = new List<WeldingPoint>();

            // Left side
            // Segment 1 (10 points, GType_1)
            string[] ls1Points = { "pLS1.1", "pLS1.2", "pLS1.3", "pLS1.4", "pLS1.5", "pLS1.6", "pLS1.7", "pLS1.8", "pLS1.9", "pLS1.10" };
            foreach (var point in ls1Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 1,
                    CompatibleGunTypes = new List<string> { "GType_1" },
                    BelongsTo = "Tobj",
                    Side = "Left"
                });

            // Segment 2 (5 points, GType_1/GType_2)
            string[] ls2Points = { "pLS2.1", "pLS2.2", "pLS2.3", "pLS2.4", "pLS2.5" };
            foreach (var point in ls2Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 2,
                    CompatibleGunTypes = new List<string> { "GType_1", "GType_2" },
                    BelongsTo = "Tobj",
                    Side = "Left"
                });

            // Segment 3 (5 points, GType_1/GType_2)
            string[] ls3Points = { "pLS3.1", "pLS3.2", "pLS3.3", "pLS3.4", "pLS3.5" };
            foreach (var point in ls3Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 3,
                    CompatibleGunTypes = new List<string> { "GType_1", "GType_2" },
                    BelongsTo = "Tobj",
                    Side = "Left"
                });

            // Segment 4 (10 points, GType_3)
            string[] ls4Points = { "pLS4.1", "pLS4.2", "pLS4.3", "pLS4.4", "pLS4.5", "pLS4.6", "pLS4.7", "pLS4.8", "pLS4.9", "pLS4.10" };
            foreach (var point in ls4Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 4,
                    CompatibleGunTypes = new List<string> { "GType_3" },
                    BelongsTo = "Tobj",
                    Side = "Left"
                });

            // Segment 5 (10 points, GType_3)
            string[] ls5Points = { "pLS5.1", "pLS5.2", "pLS5.3", "pLS5.4", "pLS5.5", "pLS5.6", "pLS5.7", "pLS5.8", "pLS5.9", "pLS5.10" };
            foreach (var point in ls5Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 5,
                    CompatibleGunTypes = new List<string> { "GType_3" },
                    BelongsTo = "Tobj",
                    Side = "Left"
                });

            // Right side
            // Segment 6 (10 points, GType_1) - symmetrical to segment 1
            string[] rs6Points = { "pRS6.1", "pRS6.2", "pRS6.3", "pRS6.4", "pRS6.5", "pRS6.6", "pRS6.7", "pRS6.8", "pRS6.9", "pRS6.10" };
            foreach (var point in rs6Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 6,
                    CompatibleGunTypes = new List<string> { "GType_1" },
                    BelongsTo = "Tobj",
                    Side = "Right"
                });

            // Segment 7 (5 points, GType_1/GType_2) - symmetrical to segment 2
            string[] rs7Points = { "pRS7.1", "pRS7.2", "pRS7.3", "pRS7.4", "pRS7.5" };
            foreach (var point in rs7Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 7,
                    CompatibleGunTypes = new List<string> { "GType_1", "GType_2" },
                    BelongsTo = "Tobj",
                    Side = "Right"
                });

            // Segment 8 (5 points, GType_1/GType_2) - symmetrical to segment 3
            string[] rs8Points = { "pRS8.1", "pRS8.2", "pRS8.3", "pRS8.4", "pRS8.5" };
            foreach (var point in rs8Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 8,
                    CompatibleGunTypes = new List<string> { "GType_1", "GType_2" },
                    BelongsTo = "Tobj",
                    Side = "Right"
                });

            // Segment 9 (10 points, GType_3) - symmetrical to segment 4
            string[] rs9Points = { "pRS9.1", "pRS9.2", "pRS9.3", "pRS9.4", "pRS9.5", "pRS9.6", "pRS9.7", "pRS9.8", "pRS9.9", "pRS9.10" };
            foreach (var point in rs9Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 9,
                    CompatibleGunTypes = new List<string> { "GType_3" },
                    BelongsTo = "Tobj",
                    Side = "Right"
                });

            // Segment 10 (10 points, GType_3) - symmetrical to segment 5
            string[] rs10Points = { "pRS10.1", "pRS10.2", "pRS10.3", "pRS10.4", "pRS10.5", "pRS10.6", "pRS10.7", "pRS10.8", "pRS10.9", "pRS10.10" };
            foreach (var point in rs10Points)
                points.Add(new WeldingPoint
                {
                    Name = point,
                    Segment = 10,
                    CompatibleGunTypes = new List<string> { "GType_3" },
                    BelongsTo = "Tobj",
                    Side = "Right"
                });

            return points;
        }

        static List<Robot> LoadRobots()
        {
            return new List<Robot>
            {
                new Robot { Name = "R1", Side = "Left", GunName = "Gun1", Object = "Tobj",
                           HomeX = 3000, HomeY = 5000, HomeZ = 1000, Speed = 3000 },
                new Robot { Name = "R3", Side = "Left", GunName = "Gun2", Object = "Tobj",
                           HomeX = 3000, HomeY = 5000, HomeZ = 1000, Speed = 3000 },
                new Robot { Name = "R5", Side = "Left", GunName = "Gun3", Object = "Tobj",
                           HomeX = 3000, HomeY = 5000, HomeZ = 1000, Speed = 3000 },
                new Robot { Name = "R7", Side = "Left", GunName = "Gun4", Object = "Tobj",
                           HomeX = 3000, HomeY = 5000, HomeZ = 1000, Speed = 3000 },
                new Robot { Name = "R2", Side = "Right", GunName = "Gun5", Object = "Tobj",
                           HomeX = 3000, HomeY = 1000, HomeZ = 1000, Speed = 3000 },
                new Robot { Name = "R4", Side = "Right", GunName = "Gun6", Object = "Tobj",
                           HomeX = 3000, HomeY = 1000, HomeZ = 1000, Speed = 3000 },
                new Robot { Name = "R6", Side = "Right", GunName = "Gun7", Object = "Tobj",
                           HomeX = 3000, HomeY = 1000, HomeZ = 1000, Speed = 3000 },
                new Robot { Name = "R8", Side = "Right", GunName = "Gun8", Object = "Tobj",
                           HomeX = 3000, HomeY = 1000, HomeZ = 1000, Speed = 3000 }
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

            // Assign robots to posts
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
                new WeldingGun { Name = "Gun1", Type = "GType_1", WeldTimePerPoint = 2, Weight = 150, Wear = 0 },
                new WeldingGun { Name = "Gun2", Type = "GType_2", WeldTimePerPoint = 2, Weight = 170, Wear = 10 },
                new WeldingGun { Name = "Gun3", Type = "GType_3", WeldTimePerPoint = 2, Weight = 120, Wear = 40 },
                new WeldingGun { Name = "Gun4", Type = "GType_3", WeldTimePerPoint = 2, Weight = 145, Wear = 25 },
                new WeldingGun { Name = "Gun5", Type = "GType_1", WeldTimePerPoint = 2, Weight = 150, Wear = 10 },
                new WeldingGun { Name = "Gun6", Type = "GType_2", WeldTimePerPoint = 2, Weight = 170, Wear = 5 },
                new WeldingGun { Name = "Gun7", Type = "GType_3", WeldTimePerPoint = 2, Weight = 120, Wear = 15 },
                new WeldingGun { Name = "Gun8", Type = "GType_3", WeldTimePerPoint = 2, Weight = 145, Wear = 0 }
            };
        }

        // 2. Assign gun types to robots
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

        // 3. Display initial data
        static void DisplayInitialData(List<WeldingPoint> points, List<Robot> robots, List<WeldingGun> guns)
        {
            Console.WriteLine("INITIAL DATA:");
            Console.WriteLine("=============\n");

            Console.WriteLine($"1. Total welding points: {points.Count}");
            Console.WriteLine($"   - Left side: {points.Count(p => p.Side == "Left")} points");
            Console.WriteLine($"   - Right side: {points.Count(p => p.Side == "Right")} points");

            Console.WriteLine("\n2. Distribution by segments:");
            var segments = points.GroupBy(p => p.Segment)
                                .OrderBy(g => g.Key)
                                .Select(g => new {
                                    Segment = g.Key,
                                    Count = g.Count(),
                                    Side = g.First().Side,
                                    Types = string.Join("/", g.First().CompatibleGunTypes)
                                });

            foreach (var seg in segments)
            {
                Console.WriteLine($"   Segment {seg.Segment}: {seg.Count} points ({seg.Side}), Gun types: {seg.Types}");
            }

            Console.WriteLine("\n3. Robots and their guns:");
            foreach (var robot in robots.OrderBy(r => r.Name))
            {
                Console.WriteLine($"   {robot.Name}: {robot.Side} side, Guns: {robot.GunName} ({robot.GunType})");
            }

            Console.WriteLine("\n4. Posts and robots:");
            foreach (var robot in robots.OrderBy(r => r.Name))
            {
                string post = robot.Name switch
                {
                    "R1" or "R2" => "10",
                    "R3" or "R4" => "20",
                    "R5" or "R6" => "30",
                    "R7" or "R8" => "40",
                    _ => "?"
                };
                Console.WriteLine($"   {robot.Name} → Post {post}");
            }

            Console.WriteLine("\n5. Welding guns:");
            foreach (var gun in guns.OrderBy(g => g.Name))
            {
                Console.WriteLine($"   {gun.Name}: Type {gun.Type}, Wear {gun.Wear}%, Weight {gun.Weight}kg");
            }

            Console.WriteLine("\n" + new string('=', 60));
        }

        // 4. Algorithm for finding optimal distribution
        static Distribution FindOptimalDistribution(
            List<WeldingPoint> allPoints,
            List<Robot> robots,
            List<Post> posts,
            List<WeldingGun> guns)
        {
            Console.WriteLine("Searching for optimal distribution...");
            Console.WriteLine("Algorithm: Genetic search with balancing optimization\n");

            // Separate points by sides
            var leftPoints = allPoints.Where(p => p.Side == "Left").ToList();
            var rightPoints = allPoints.Where(p => p.Side == "Right").ToList();

            var leftRobots = robots.Where(r => r.Side == "Left").ToList();
            var rightRobots = robots.Where(r => r.Side == "Right").ToList();

            // Create initial population of distributions
            var population = new List<Distribution>();
            int populationSize = 50;

            Console.WriteLine($"Generating initial population ({populationSize} variants):");

            for (int i = 0; i < populationSize; i++)
            {
                Distribution dist;

                if (i < 10)
                {
                    // First 10 - random distributions
                    dist = GenerateRandomDistribution(allPoints, robots, $"Random #{i + 1}");
                }
                else if (i < 20)
                {
                    // Next 10 - compatibility-based distributions
                    dist = GenerateCompatibilityDistribution(allPoints, robots, $"By compatibility #{i - 9}");
                }
                else if (i < 30)
                {
                    // Next 10 - load balanced distributions
                    dist = GenerateLoadBalancedDistribution(allPoints, robots, $"Balanced #{i - 19}");
                }
                else
                {
                    // Rest - mixed strategies
                    dist = GenerateMixedDistribution(allPoints, robots, $"Mixed #{i - 29}");
                }

                population.Add(dist);

                if (i < 5) // Show first 5
                    Console.WriteLine($"  Variant {i + 1}: {dist.Strategy}, Score: {dist.Score:F2}");
            }

            // Evolutionary algorithm: 20 generations
            int generations = 20;
            Distribution bestDistribution = null;
            double bestScore = double.MaxValue;

            Console.WriteLine($"\nEvolutionary optimization ({generations} generations):");

            for (int gen = 0; gen < generations; gen++)
            {
                // Evaluate all distributions
                foreach (var dist in population)
                {
                    dist.Score = CalculateDistributionScore(dist, robots, allPoints);
                }

                // Sort by quality
                population.Sort((a, b) => a.Score.CompareTo(b.Score));

                // Best distribution in this generation
                var currentBest = population[0];

                if (currentBest.Score < bestScore)
                {
                    bestScore = currentBest.Score;
                    bestDistribution = currentBest;
                    Console.WriteLine($"  Generation {gen + 1}: Best score = {bestScore:F2}, Strategy: {bestDistribution.Strategy}");
                }

                // Create new generation
                var newGeneration = new List<Distribution>();

                // Elitism: keep best 10%
                int eliteCount = Math.Max(1, populationSize / 10);
                newGeneration.AddRange(population.Take(eliteCount));

                // Crossover and mutation
                Random rand = new Random(DateTime.Now.Millisecond + gen);
                while (newGeneration.Count < populationSize)
                {
                    // Parent selection (tournament selection)
                    var parent1 = TournamentSelection(population, rand, 3);
                    var parent2 = TournamentSelection(population, rand, 3);

                    // Crossover
                    var child = Crossover(parent1, parent2, allPoints, robots, $"Crossover Gen{gen}");

                    // Mutation
                    if (rand.NextDouble() < 0.3) // 30% mutation chance
                    {
                        Mutate(child, allPoints, robots, rand);
                        child.Strategy = $"Mutated Gen{gen}";
                    }

                    newGeneration.Add(child);
                }

                population = newGeneration;
            }

            // Additional local optimization of best distribution
            Console.WriteLine("\nLocal optimization of best distribution...");
            bestDistribution = LocalOptimization(bestDistribution, allPoints, robots);
            bestDistribution.Score = CalculateDistributionScore(bestDistribution, robots, allPoints);

            Console.WriteLine($"\nFound optimal distribution:");
            Console.WriteLine($"  Strategy: {bestDistribution.Strategy}");
            Console.WriteLine($"  Score: {bestScore:F2}");

            return bestDistribution;
        }

        // 4.1. Generate random distribution
        static Distribution GenerateRandomDistribution(List<WeldingPoint> points, List<Robot> robots, string strategy)
        {
            var distribution = new Distribution
            {
                Strategy = strategy,
                Description = "Random initial distribution"
            };

            // Clear assignments
            foreach (var robot in robots)
            {
                distribution.RobotAssignments[robot.Name] = new List<string>();
            }

            Random rand = new Random(strategy.GetHashCode());

            // Separate points by sides
            var leftPoints = points.Where(p => p.Side == "Left").ToList();
            var rightPoints = points.Where(p => p.Side == "Right").ToList();

            // Distribute left points
            DistributeRandomly(leftPoints, robots.Where(r => r.Side == "Left").ToList(), distribution, rand);

            // Distribute right points
            DistributeRandomly(rightPoints, robots.Where(r => r.Side == "Right").ToList(), distribution, rand);

            return distribution;
        }

        static void DistributeRandomly(List<WeldingPoint> points, List<Robot> robots, Distribution distribution, Random rand)
        {
            // Shuffle points
            var shuffledPoints = points.OrderBy(x => rand.Next()).ToList();

            foreach (var point in shuffledPoints)
            {
                // Find all compatible robots
                var compatibleRobots = robots.Where(r =>
                    point.CompatibleGunTypes.Contains(r.GunType)).ToList();

                if (compatibleRobots.Count == 0) continue;

                // Choose random robot from compatible ones
                var selectedRobot = compatibleRobots[rand.Next(compatibleRobots.Count)];
                distribution.RobotAssignments[selectedRobot.Name].Add(point.Name);
            }
        }

        // 4.2. Compatibility-based distribution
        static Distribution GenerateCompatibilityDistribution(List<WeldingPoint> points, List<Robot> robots, string strategy)
        {
            var distribution = new Distribution
            {
                Strategy = strategy,
                Description = "Distribution based on maximum compatibility"
            };

            // Clear assignments
            foreach (var robot in robots)
            {
                distribution.RobotAssignments[robot.Name] = new List<string>();
            }

            // Separate points by sides
            var leftPoints = points.Where(p => p.Side == "Left").ToList();
            var rightPoints = points.Where(p => p.Side == "Right").ToList();

            // Group points by compatibility types
            var leftGroups = GroupPointsByCompatibility(leftPoints);
            var rightGroups = GroupPointsByCompatibility(rightPoints);

            // Distribute left points
            DistributeByCompatibility(leftGroups, robots.Where(r => r.Side == "Left").ToList(), distribution);

            // Distribute right points
            DistributeByCompatibility(rightGroups, robots.Where(r => r.Side == "Right").ToList(), distribution);

            return distribution;
        }

        static Dictionary<string, List<WeldingPoint>> GroupPointsByCompatibility(List<WeldingPoint> points)
        {
            var groups = new Dictionary<string, List<WeldingPoint>>();

            foreach (var point in points)
            {
                // Create key from sorted compatibility types
                var key = string.Join("&", point.CompatibleGunTypes.OrderBy(t => t));

                if (!groups.ContainsKey(key))
                    groups[key] = new List<WeldingPoint>();

                groups[key].Add(point);
            }

            return groups;
        }

        static void DistributeByCompatibility(
            Dictionary<string, List<WeldingPoint>> groups,
            List<Robot> robots,
            Distribution distribution)
        {
            // Sort groups by size (descending)
            var sortedGroups = groups.OrderByDescending(g => g.Value.Count).ToList();

            foreach (var group in sortedGroups)
            {
                var points = group.Value;
                var gunTypes = group.Key.Split('&').ToList();

                // Find robots compatible with this group
                var compatibleRobots = robots.Where(r => gunTypes.Contains(r.GunType)).ToList();

                if (compatibleRobots.Count == 0) continue;

                // Sort robots by current load
                compatibleRobots.Sort((a, b) =>
                    distribution.RobotAssignments[a.Name].Count.CompareTo(
                    distribution.RobotAssignments[b.Name].Count));

                // Distribute points
                foreach (var point in points)
                {
                    // Choose robot with minimum load
                    var selectedRobot = compatibleRobots[0];
                    distribution.RobotAssignments[selectedRobot.Name].Add(point.Name);

                    // Re-sort (since load changed)
                    compatibleRobots.Sort((a, b) =>
                        distribution.RobotAssignments[a.Name].Count.CompareTo(
                        distribution.RobotAssignments[b.Name].Count));
                }
            }
        }

        // 4.3. Load-balanced distribution
        static Distribution GenerateLoadBalancedDistribution(List<WeldingPoint> points, List<Robot> robots, string strategy)
        {
            var distribution = new Distribution
            {
                Strategy = strategy,
                Description = "Distribution with load balancing"
            };

            // Clear assignments
            foreach (var robot in robots)
            {
                distribution.RobotAssignments[robot.Name] = new List<string>();
            }

            // Separate points by sides
            var leftPoints = points.Where(p => p.Side == "Left").ToList();
            var rightPoints = points.Where(p => p.Side == "Right").ToList();

            // Distribute with load balancing
            DistributeWithLoadBalancing(leftPoints, robots.Where(r => r.Side == "Left").ToList(), distribution);
            DistributeWithLoadBalancing(rightPoints, robots.Where(r => r.Side == "Right").ToList(), distribution);

            return distribution;
        }

        static void DistributeWithLoadBalancing(List<WeldingPoint> points, List<Robot> robots, Distribution distribution)
        {
            // Group robots by gun types
            var robotsByType = robots.GroupBy(r => r.GunType)
                                    .ToDictionary(g => g.Key, g => g.ToList());

            // For each gun type
            foreach (var robotTypeGroup in robotsByType)
            {
                string gunType = robotTypeGroup.Key;
                var typeRobots = robotTypeGroup.Value;

                // Find points compatible with this type
                var compatiblePoints = points.Where(p =>
                    p.CompatibleGunTypes.Contains(gunType) &&
                    !distribution.RobotAssignments.Values.SelectMany(x => x).Contains(p.Name))
                    .ToList();

                if (compatiblePoints.Count == 0 || typeRobots.Count == 0) continue;

                // Distribute points evenly
                int pointsPerRobot = compatiblePoints.Count / typeRobots.Count;
                int remainder = compatiblePoints.Count % typeRobots.Count;

                int currentIndex = 0;
                for (int i = 0; i < typeRobots.Count; i++)
                {
                    int takeCount = pointsPerRobot + (i < remainder ? 1 : 0);
                    if (currentIndex + takeCount > compatiblePoints.Count) break;

                    var robotPoints = compatiblePoints.Skip(currentIndex).Take(takeCount);
                    foreach (var point in robotPoints)
                    {
                        distribution.RobotAssignments[typeRobots[i].Name].Add(point.Name);
                    }
                    currentIndex += takeCount;
                }
            }

            // Handle remaining points
            var unassignedPoints = points.Where(p =>
                !distribution.RobotAssignments.Values.SelectMany(x => x).Contains(p.Name)).ToList();

            foreach (var point in unassignedPoints)
            {
                // Find any compatible robot with minimum load
                var compatibleRobots = robots.Where(r =>
                    point.CompatibleGunTypes.Contains(r.GunType)).ToList();

                if (compatibleRobots.Count == 0) continue;

                compatibleRobots.Sort((a, b) =>
                    distribution.RobotAssignments[a.Name].Count.CompareTo(
                    distribution.RobotAssignments[b.Name].Count));

                distribution.RobotAssignments[compatibleRobots[0].Name].Add(point.Name);
            }
        }

        // 4.4. Mixed distribution
        static Distribution GenerateMixedDistribution(List<WeldingPoint> points, List<Robot> robots, string strategy)
        {
            // Combine different strategies
            Random rand = new Random(strategy.GetHashCode());

            // First use compatibility-based distribution
            var distribution = GenerateCompatibilityDistribution(points, robots, strategy);

            // Then apply random movements for balancing
            BalanceDistribution(distribution, robots, points, rand, 10);

            distribution.Description = "Mixed strategy with balancing";
            return distribution;
        }

        static void BalanceDistribution(Distribution distribution, List<Robot> robots,
                                       List<WeldingPoint> points, Random rand, int iterations)
        {
            for (int i = 0; i < iterations; i++)
            {
                // Find robot with maximum load
                var overloadedRobot = robots
                    .Where(r => distribution.RobotAssignments[r.Name].Count > 0)
                    .OrderByDescending(r => distribution.RobotAssignments[r.Name].Count)
                    .FirstOrDefault();

                if (overloadedRobot == null) break;

                // Find robot with minimum load
                var underloadedRobot = robots
                    .Where(r => r.Side == overloadedRobot.Side &&
                                r.Name != overloadedRobot.Name)
                    .OrderBy(r => distribution.RobotAssignments[r.Name].Count)
                    .FirstOrDefault();

                if (underloadedRobot == null) break;

                // Try to move a point
                var pointsToMove = distribution.RobotAssignments[overloadedRobot.Name]
                    .Where(p =>
                    {
                        var point = points.FirstOrDefault(pt => pt.Name == p);
                        return point != null && point.CompatibleGunTypes.Contains(underloadedRobot.GunType);
                    })
                    .ToList();

                if (pointsToMove.Count == 0) continue;

                // Move random point
                string pointToMove = pointsToMove[rand.Next(pointsToMove.Count)];
                distribution.RobotAssignments[overloadedRobot.Name].Remove(pointToMove);
                distribution.RobotAssignments[underloadedRobot.Name].Add(pointToMove);
            }
        }

        // 4.5. Tournament selection
        static Distribution TournamentSelection(List<Distribution> population, Random rand, int tournamentSize)
        {
            var tournament = new List<Distribution>();

            for (int i = 0; i < tournamentSize; i++)
            {
                int index = rand.Next(population.Count);
                tournament.Add(population[index]);
            }

            // Return best from tournament
            return tournament.OrderBy(d => d.Score).First();
        }

        // 4.6. Crossover
        static Distribution Crossover(Distribution parent1, Distribution parent2,
                                     List<WeldingPoint> points, List<Robot> robots, string strategy)
        {
            var child = new Distribution
            {
                Strategy = strategy,
                Description = $"Child of {parent1.Strategy} and {parent2.Strategy}"
            };

            // Clear assignments
            foreach (var robot in robots)
            {
                child.RobotAssignments[robot.Name] = new List<string>();
            }

            Random rand = new Random(strategy.GetHashCode());

            // For each robot randomly choose points from one of the parents
            foreach (var robot in robots)
            {
                var parent = rand.NextDouble() < 0.5 ? parent1 : parent2;

                if (parent.RobotAssignments.ContainsKey(robot.Name))
                {
                    // Take points from selected parent
                    foreach (var pointName in parent.RobotAssignments[robot.Name])
                    {
                        child.RobotAssignments[robot.Name].Add(pointName);
                    }
                }
            }

            // Remove duplicates (if point ended up with multiple robots)
            RemoveDuplicates(child, points, robots);

            // Add missing points
            AddMissingPoints(child, points, robots);

            return child;
        }

        // 4.7. Mutation
        static void Mutate_OptimizeSegments(Distribution distribution, List<WeldingPoint> points,
                                           List<Robot> robots, Random rand)
        {
            // Analyze current segment distribution
            Dictionary<int, List<string>> segmentToRobots = new Dictionary<int, List<string>>();
            Dictionary<int, List<string>> segmentPoints = new Dictionary<int, List<string>>();

            // Group points by segments
            foreach (var point in points)
            {
                if (!segmentPoints.ContainsKey(point.Segment))
                    segmentPoints[point.Segment] = new List<string>();
                segmentPoints[point.Segment].Add(point.Name);
            }

            // Find segments split between multiple robots
            foreach (var segment in segmentPoints.Keys)
            {
                segmentToRobots[segment] = new List<string>();
                foreach (var robot in robots)
                {
                    if (distribution.RobotAssignments.ContainsKey(robot.Name))
                    {
                        var robotSegmentPoints = distribution.RobotAssignments[robot.Name]
                            .Where(p => points.First(pt => pt.Name == p).Segment == segment)
                            .Count();

                        if (robotSegmentPoints > 0)
                        {
                            segmentToRobots[segment].Add(robot.Name);
                        }
                    }
                }
            }

            // Find segments split between multiple robots
            var splitSegments = segmentToRobots.Where(kvp => kvp.Value.Count > 1).ToList();

            if (splitSegments.Count == 0) return;

            // Choose random split segment
            var selectedSegment = splitSegments[rand.Next(splitSegments.Count)];
            int segmentId = selectedSegment.Key;

            // Find all robots that have points of this segment
            var robotsWithSegment = selectedSegment.Value;

            // Find all compatible robots for this segment
            var segmentSide = points.First(p => p.Segment == segmentId).Side;
            var segmentGunTypes = points.First(p => p.Segment == segmentId).CompatibleGunTypes;

            var compatibleRobots = robots.Where(r =>
                r.Side == segmentSide &&
                segmentGunTypes.Contains(r.GunType)).ToList();

            if (compatibleRobots.Count == 0) return;

            // Choose one robot to receive the entire segment
            var targetRobot = compatibleRobots[rand.Next(compatibleRobots.Count)];

            // Collect all segment points
            var allSegmentPoints = points.Where(p => p.Segment == segmentId)
                                        .Select(p => p.Name).ToList();

            // Remove segment points from all robots
            foreach (var robot in robots)
            {
                if (distribution.RobotAssignments.ContainsKey(robot.Name))
                {
                    distribution.RobotAssignments[robot.Name] = distribution.RobotAssignments[robot.Name]
                        .Where(p => !allSegmentPoints.Contains(p))
                        .ToList();
                }
            }

            // Assign all segment points to target robot
            distribution.RobotAssignments[targetRobot.Name].AddRange(allSegmentPoints);
        }

        // Update main mutation function
        static void Mutate(Distribution distribution, List<WeldingPoint> points, List<Robot> robots, Random rand)
        {
            // Choose random number of mutations (1-4)
            int mutationCount = rand.Next(1, 5);

            for (int m = 0; m < mutationCount; m++)
            {
                // Mutation type (added segment optimization)
                int mutationType = rand.Next(5);

                switch (mutationType)
                {
                    case 0: // Move point between robots
                        Mutate_MovePoint(distribution, points, robots, rand);
                        break;

                    case 1: // Swap points between robots
                        Mutate_SwapPoints(distribution, points, robots, rand);
                        break;

                    case 2: // Redistribute segment
                        Mutate_RedistributeSegment(distribution, points, robots, rand);
                        break;

                    case 3: // Random change
                        Mutate_RandomChange(distribution, points, robots, rand);
                        break;

                    case 4: // Segment optimization
                        Mutate_OptimizeSegments(distribution, points, robots, rand);
                        break;
                }
            }

            // Clean duplicates after mutation
            RemoveDuplicates(distribution, points, robots);
            AddMissingPoints(distribution, points, robots);
        }

        static void Mutate_MovePoint(Distribution distribution, List<WeldingPoint> points,
                                    List<Robot> robots, Random rand)
        {
            // Find robot with maximum load
            var overloadedRobot = robots
                .Where(r => distribution.RobotAssignments[r.Name].Count > 0)
                .OrderByDescending(r => distribution.RobotAssignments[r.Name].Count)
                .FirstOrDefault();

            if (overloadedRobot == null) return;

            // Choose random point from this robot
            var pointsList = distribution.RobotAssignments[overloadedRobot.Name];
            if (pointsList.Count == 0) return;

            string pointToMove = pointsList[rand.Next(pointsList.Count)];
            var point = points.FirstOrDefault(p => p.Name == pointToMove);
            if (point == null) return;

            // Find compatible robot with minimum load
            var compatibleRobots = robots
                .Where(r => r.Name != overloadedRobot.Name &&
                            point.CompatibleGunTypes.Contains(r.GunType) &&
                            r.Side == point.Side)
                .OrderBy(r => distribution.RobotAssignments[r.Name].Count)
                .ToList();

            if (compatibleRobots.Count == 0) return;

            // Move point
            distribution.RobotAssignments[overloadedRobot.Name].Remove(pointToMove);
            distribution.RobotAssignments[compatibleRobots[0].Name].Add(pointToMove);
        }

        static void Mutate_SwapPoints(Distribution distribution, List<WeldingPoint> points,
                                     List<Robot> robots, Random rand)
        {
            // Choose two random robots
            var robot1 = robots[rand.Next(robots.Count)];
            var robot2 = robots.Where(r => r != robot1).ElementAtOrDefault(rand.Next(robots.Count - 1));

            if (robot2 == null ||
                distribution.RobotAssignments[robot1.Name].Count == 0 ||
                distribution.RobotAssignments[robot2.Name].Count == 0)
                return;

            // Choose random points from each robot
            string point1 = distribution.RobotAssignments[robot1.Name][rand.Next(distribution.RobotAssignments[robot1.Name].Count)];
            string point2 = distribution.RobotAssignments[robot2.Name][rand.Next(distribution.RobotAssignments[robot2.Name].Count)];

            var p1 = points.FirstOrDefault(p => p.Name == point1);
            var p2 = points.FirstOrDefault(p => p.Name == point2);

            if (p1 == null || p2 == null) return;

            // Check compatibility
            bool canSwap = p1.CompatibleGunTypes.Contains(robot2.GunType) &&
                           p2.CompatibleGunTypes.Contains(robot1.GunType) &&
                           p1.Side == robot2.Side &&
                           p2.Side == robot1.Side;

            if (canSwap)
            {
                // Swap points
                distribution.RobotAssignments[robot1.Name].Remove(point1);
                distribution.RobotAssignments[robot2.Name].Remove(point2);
                distribution.RobotAssignments[robot1.Name].Add(point2);
                distribution.RobotAssignments[robot2.Name].Add(point1);
            }
        }

        static void Mutate_RedistributeSegment(Distribution distribution, List<WeldingPoint> points,
                                              List<Robot> robots, Random rand)
        {
            // Choose random segment
            var segments = points.Select(p => p.Segment).Distinct().ToList();
            int segment = segments[rand.Next(segments.Count)];

            var segmentPoints = points.Where(p => p.Segment == segment).ToList();
            var segmentSide = segmentPoints.First().Side;

            // Find all robots currently having points of this segment
            var currentRobots = robots.Where(r =>
                distribution.RobotAssignments[r.Name].Any(p =>
                    segmentPoints.Any(sp => sp.Name == p))).ToList();

            if (currentRobots.Count == 0) return;

            // Find all compatible robots for this segment
            var compatibleRobots = robots.Where(r =>
                r.Side == segmentSide &&
                segmentPoints[0].CompatibleGunTypes.Contains(r.GunType)).ToList();

            if (compatibleRobots.Count == 0) return;

            // Remove all segment points
            foreach (var point in segmentPoints)
            {
                foreach (var robot in robots)
                {
                    distribution.RobotAssignments[robot.Name].Remove(point.Name);
                }
            }

            // Redistribute segment evenly
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
                    distribution.RobotAssignments[compatibleRobots[i].Name].Add(point.Name);
                }
                currentIndex += takeCount;
            }
        }

        static void Mutate_RandomChange(Distribution distribution, List<WeldingPoint> points,
                                       List<Robot> robots, Random rand)
        {
            // Choose random point
            var allAssigned = distribution.RobotAssignments.Values.SelectMany(x => x).ToList();
            if (allAssigned.Count == 0) return;

            string randomPoint = allAssigned[rand.Next(allAssigned.Count)];
            var point = points.FirstOrDefault(p => p.Name == randomPoint);
            if (point == null) return;

            // Find current robot of this point
            string currentRobotName = distribution.RobotAssignments
                .First(kvp => kvp.Value.Contains(randomPoint)).Key;

            var currentRobot = robots.FirstOrDefault(r => r.Name == currentRobotName);
            if (currentRobot == null) return;

            // Find all compatible robots
            var compatibleRobots = robots.Where(r =>
                r.Name != currentRobotName &&
                r.Side == point.Side &&
                point.CompatibleGunTypes.Contains(r.GunType)).ToList();

            if (compatibleRobots.Count == 0) return;

            // Choose random compatible robot
            var newRobot = compatibleRobots[rand.Next(compatibleRobots.Count)];

            // Move point
            distribution.RobotAssignments[currentRobotName].Remove(randomPoint);
            distribution.RobotAssignments[newRobot.Name].Add(randomPoint);
        }

        // 4.8. Remove duplicates
        static void RemoveDuplicates(Distribution distribution, List<WeldingPoint> points, List<Robot> robots)
        {
            var pointCount = new Dictionary<string, int>();

            // Count how many times each point is assigned
            foreach (var assignment in distribution.RobotAssignments.Values)
            {
                foreach (var pointName in assignment)
                {
                    if (!pointCount.ContainsKey(pointName))
                        pointCount[pointName] = 0;
                    pointCount[pointName]++;
                }
            }

            // Remove duplicates
            var duplicatePoints = pointCount.Where(kvp => kvp.Value > 1).Select(kvp => kvp.Key).ToList();

            foreach (var pointName in duplicatePoints)
            {
                // Keep point only with one robot (with minimum load)
                var robotsWithPoint = robots.Where(r =>
                    distribution.RobotAssignments[r.Name].Contains(pointName)).ToList();

                // Choose robot with minimum load
                robotsWithPoint.Sort((a, b) =>
                    distribution.RobotAssignments[a.Name].Count.CompareTo(
                    distribution.RobotAssignments[b.Name].Count));

                var keeper = robotsWithPoint[0];

                // Remove from others
                foreach (var robot in robotsWithPoint.Skip(1))
                {
                    distribution.RobotAssignments[robot.Name].Remove(pointName);
                }
            }
        }

        // 4.9. Add missing points
        static void AddMissingPoints(Distribution distribution, List<WeldingPoint> points, List<Robot> robots)
        {
            var allAssigned = distribution.RobotAssignments.Values.SelectMany(x => x).Distinct().ToList();
            var unassignedPoints = points.Where(p => !allAssigned.Contains(p.Name)).ToList();

            foreach (var point in unassignedPoints)
            {
                // Find compatible robot with minimum load
                var compatibleRobots = robots.Where(r =>
                    r.Side == point.Side &&
                    point.CompatibleGunTypes.Contains(r.GunType)).ToList();

                if (compatibleRobots.Count == 0) continue;

                compatibleRobots.Sort((a, b) =>
                    distribution.RobotAssignments[a.Name].Count.CompareTo(
                    distribution.RobotAssignments[b.Name].Count));

                distribution.RobotAssignments[compatibleRobots[0].Name].Add(point.Name);
            }
        }

        // 4.10. Local optimization
        static Distribution LocalOptimization(Distribution distribution, List<WeldingPoint> points, List<Robot> robots)
        {
            var optimized = new Distribution
            {
                Strategy = distribution.Strategy + " (optimized)",
                Description = "Locally optimized distribution considering segments"
            };

            // Copy current distribution
            foreach (var kvp in distribution.RobotAssignments)
            {
                optimized.RobotAssignments[kvp.Key] = new List<string>(kvp.Value);
            }

            Random rand = new Random();
            double currentScore = CalculateDistributionScore(optimized, robots, points);
            int iterationsWithoutImprovement = 0;

            // Perform iterative optimization with focus on segments
            for (int iteration = 0; iteration < 100; iteration++)
            {
                bool improved = false;

                // Try to optimize each segment
                var segments = points.Select(p => p.Segment).Distinct().ToList();

                foreach (var segment in segments.OrderBy(s => rand.Next()))
                {
                    // Save current state
                    var backup = new Distribution();
                    foreach (var kvp in optimized.RobotAssignments)
                    {
                        backup.RobotAssignments[kvp.Key] = new List<string>(kvp.Value);
                    }

                    // Try to optimize this segment
                    OptimizeSegment(optimized, segment, points, robots, rand);

                    // Check if improved
                    double newScore = CalculateDistributionScore(optimized, robots, points);

                    if (newScore < currentScore)
                    {
                        currentScore = newScore;
                        improved = true;
                        iterationsWithoutImprovement = 0;
                    }
                    else
                    {
                        // Revert if not improved
                        optimized.RobotAssignments = backup.RobotAssignments;
                    }
                }

                if (!improved)
                {
                    iterationsWithoutImprovement++;
                    if (iterationsWithoutImprovement > 5) break;
                }

                // Also try regular mutations
                for (int i = 0; i < 10; i++)
                {
                    var backup = new Distribution();
                    foreach (var kvp in optimized.RobotAssignments)
                    {
                        backup.RobotAssignments[kvp.Key] = new List<string>(kvp.Value);
                    }

                    Mutate(optimized, points, robots, rand);

                    double newScore = CalculateDistributionScore(optimized, robots, points);

                    if (newScore < currentScore)
                    {
                        currentScore = newScore;
                        improved = true;
                        iterationsWithoutImprovement = 0;
                    }
                    else
                    {
                        optimized.RobotAssignments = backup.RobotAssignments;
                    }
                }
            }

            return optimized;
        }
        static void OptimizeSegment(Distribution distribution, int segment,
                           List<WeldingPoint> points, List<Robot> robots, Random rand)
        {
            var segmentPoints = points.Where(p => p.Segment == segment).ToList();
            if (segmentPoints.Count == 0) return;

            var segmentSide = segmentPoints[0].Side;
            var segmentGunTypes = segmentPoints[0].CompatibleGunTypes;

            // Find all robots compatible with this segment
            var compatibleRobots = robots.Where(r =>
                r.Side == segmentSide &&
                segmentGunTypes.Contains(r.GunType)).ToList();

            if (compatibleRobots.Count == 0) return;

            // Find current robots having points of this segment
            var currentRobots = new List<Robot>();
            foreach (var robot in robots)
            {
                if (distribution.RobotAssignments.ContainsKey(robot.Name))
                {
                    var hasSegmentPoints = distribution.RobotAssignments[robot.Name]
                        .Any(p => points.First(pt => pt.Name == p).Segment == segment);

                    if (hasSegmentPoints) currentRobots.Add(robot);
                }
            }

            // If segment already assigned entirely to one robot and robot is compatible, leave as is
            if (currentRobots.Count == 1 && compatibleRobots.Contains(currentRobots[0]))
            {
                // Check if all segment points are with this robot
                var robotSegmentPoints = distribution.RobotAssignments[currentRobots[0].Name]
                    .Where(p => points.First(pt => pt.Name == p).Segment == segment)
                    .Count();

                if (robotSegmentPoints == segmentPoints.Count) return;
            }

            // Remove all segment points from all robots
            var allSegmentPointNames = segmentPoints.Select(p => p.Name).ToList();
            foreach (var robot in robots)
            {
                if (distribution.RobotAssignments.ContainsKey(robot.Name))
                {
                    distribution.RobotAssignments[robot.Name] = distribution.RobotAssignments[robot.Name]
                        .Where(p => !allSegmentPointNames.Contains(p))
                        .ToList();
                }
            }

            // Distribute segment to one of the compatible robots
            // Choose robot with minimum load
            compatibleRobots.Sort((a, b) =>
                distribution.RobotAssignments[a.Name].Count.CompareTo(
                distribution.RobotAssignments[b.Name].Count));

            var selectedRobot = compatibleRobots[0];
            distribution.RobotAssignments[selectedRobot.Name].AddRange(allSegmentPointNames);
        }

        // 4.11. Distribution scoring function
        static double CalculateDistributionScore(Distribution distribution, List<Robot> robots, List<WeldingPoint> allPoints)
        {
            double score = 0;

            // 1. Penalty for each unassigned point
            var assigned = distribution.RobotAssignments.Values.SelectMany(x => x).Distinct().Count();
            score += (allPoints.Count - assigned) * 1000;

            // 2. Penalty for each point with multiple robots
            var allAssigned = distribution.RobotAssignments.Values.SelectMany(x => x).ToList();
            var duplicateCount = allAssigned.Count - allAssigned.Distinct().Count();
            score += duplicateCount * 1000;

            // 3. Penalty for gun type incompatibility
            foreach (var robot in robots)
            {
                if (distribution.RobotAssignments.ContainsKey(robot.Name))
                {
                    foreach (var pointName in distribution.RobotAssignments[robot.Name])
                    {
                        var point = allPoints.FirstOrDefault(p => p.Name == pointName);
                        if (point != null && !point.CompatibleGunTypes.Contains(robot.GunType))
                            score += 500;
                    }
                }
            }

            // 4. Penalty for working on wrong side
            foreach (var robot in robots)
            {
                if (distribution.RobotAssignments.ContainsKey(robot.Name))
                {
                    foreach (var pointName in distribution.RobotAssignments[robot.Name])
                    {
                        var point = allPoints.FirstOrDefault(p => p.Name == pointName);
                        if (point != null && point.Side != robot.Side)
                            score += 1000;
                    }
                }
            }

            // 5. Penalty for load imbalance between robots
            var robotLoads = robots.Select(r =>
                distribution.RobotAssignments.ContainsKey(r.Name) ?
                distribution.RobotAssignments[r.Name].Count : 0).ToList();

            if (robotLoads.Count > 0)
            {
                double avgLoad = robotLoads.Average();
                double imbalance = robotLoads.Sum(load => Math.Abs(load - avgLoad));
                score += imbalance * 20;
            }

            // 6. Penalty for distributing segments between robots
            Dictionary<int, List<string>> segmentToRobots = new Dictionary<int, List<string>>();

            foreach (var robot in robots)
            {
                if (distribution.RobotAssignments.ContainsKey(robot.Name))
                {
                    foreach (var pointName in distribution.RobotAssignments[robot.Name])
                    {
                        var point = allPoints.FirstOrDefault(p => p.Name == pointName);
                        if (point != null)
                        {
                            if (!segmentToRobots.ContainsKey(point.Segment))
                                segmentToRobots[point.Segment] = new List<string>();

                            if (!segmentToRobots[point.Segment].Contains(robot.Name))
                                segmentToRobots[point.Segment].Add(robot.Name);
                        }
                    }
                }
            }

            // Penalty for segments split between multiple robots
            foreach (var segment in segmentToRobots)
            {
                if (segment.Value.Count > 1)
                {
                    // Penalty increases with number of robots sharing segment
                    score += segment.Value.Count * 50;
                }
            }

            // 7. Bonus for complete distribution of all points
            if (assigned == allPoints.Count)
            {
                score -= 2000;
            }

            // 8. Bonus for even distribution (10 points per robot)
            int targetPointsPerRobot = allPoints.Count / robots.Count;
            int perfectRobots = robotLoads.Count(load => load == targetPointsPerRobot);
            score -= perfectRobots * 100;

            // 9. Bonus for assigning whole segments to robots
            int wholeSegmentsAssigned = 0;
            foreach (var robot in robots)
            {
                if (distribution.RobotAssignments.ContainsKey(robot.Name))
                {
                    var robotSegments = new Dictionary<int, int>();
                    foreach (var pointName in distribution.RobotAssignments[robot.Name])
                    {
                        var point = allPoints.FirstOrDefault(p => p.Name == pointName);
                        if (point != null)
                        {
                            if (!robotSegments.ContainsKey(point.Segment))
                                robotSegments[point.Segment] = 0;
                            robotSegments[point.Segment]++;
                        }
                    }

                    // Check if robot has whole segments
                    foreach (var segment in robotSegments)
                    {
                        int totalPointsInSegment = allPoints.Count(p => p.Segment == segment.Key);
                        if (segment.Value == totalPointsInSegment)
                        {
                            wholeSegmentsAssigned++;
                        }
                    }
                }
            }
            score -= wholeSegmentsAssigned * 50;

            return score;
        }

        // 5. Apply best distribution
        static void ApplyBestDistribution(List<WeldingPoint> allPoints, List<Robot> robots, Distribution bestDistribution)
        {
            Console.WriteLine("\nApplying best distribution...");

            // Clear current assignments
            foreach (var robot in robots)
            {
                robot.AssignedPoints.Clear();
            }

            // Create point dictionary for quick lookup
            var pointDict = allPoints.ToDictionary(p => p.Name);

            // Apply distribution
            foreach (var kvp in bestDistribution.RobotAssignments)
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

            Console.WriteLine("Distribution applied.");
        }

        // 6. Calculate timing parameters
        static void CalculateAllTimes(List<Robot> robots, List<WeldingGun> guns, List<Post> posts)
        {
            var gunDict = guns.ToDictionary(g => g.Name);

            // Calculate time for each robot
            foreach (var robot in robots)
            {
                if (robot.AssignedPoints.Count == 0)
                {
                    robot.CycleTime = 0;
                    continue;
                }

                // Welding time
                double weldTimePerPoint = 2.0;
                if (gunDict.TryGetValue(robot.GunName, out var gun))
                    weldTimePerPoint = gun.WeldTimePerPoint;

                // Movement time calculation (simplified, but considering point count)
                double movementTime = CalculateRobotMovementTime(robot);

                // Total time
                double weldingTime = robot.AssignedPoints.Count * weldTimePerPoint;
                robot.CycleTime = movementTime + weldingTime;
            }

            // Calculate time for each post
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

            // More accurate movement time calculation
            // Consider point count and assume robot moves along optimal route

            // Base time for moving between Home and first point
            double baseTime = 2.0; // seconds

            // Time between points
            // Assume average distance between points in segment - 300 mm
            double timeBetweenPoints = 0.5; // seconds

            // Time to return to Home position
            double returnTime = 1.5; // seconds

            // Total movement time
            return baseTime + (robot.AssignedPoints.Count - 1) * timeBetweenPoints + returnTime;
        }

        // 7. Display results
        static void DisplayResults(List<WeldingPoint> allPoints, List<Robot> robots,
                                  List<Post> posts, List<WeldingGun> guns, Distribution bestDistribution)
        {
            Console.WriteLine("\n" + new string('=', 70));
            Console.WriteLine("OPTIMAL DISTRIBUTION RESULTS");
            Console.WriteLine(new string('=', 70) + "\n");

            // Table 1: Distribution of points by robots
            Console.WriteLine("TABLE 1: Distribution of welding points by robots");
            Console.WriteLine(new string('=', 100));
            Console.WriteLine("Post | Side    | Robot | Part Segments | Welding Points | Count | Robot Time(sec)");
            Console.WriteLine(new string('-', 100));

            foreach (var post in posts.OrderBy(p => p.Number))
            {
                var allPostRobots = post.LeftRobots.Concat(post.RightRobots).OrderBy(r => r.Name);

                foreach (var robot in allPostRobots)
                {
                    // Get unique segments
                    var segments = robot.AssignedPoints
                        .Select(p => p.Segment)
                        .Distinct()
                        .OrderBy(s => s)
                        .ToList();

                    string segmentsStr = string.Join(",", segments);

                    // Get point names
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

            // Table 2: Post summary
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

            // Table 3: Welding gun assignment
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

            // Information about best distribution
            Console.WriteLine("\nINFORMATION ABOUT FOUND DISTRIBUTION:");
            Console.WriteLine(new string('=', 40));
            Console.WriteLine($"Search strategy: {bestDistribution.Strategy}");
            Console.WriteLine($"Description: {bestDistribution.Description}");
            Console.WriteLine($"Distribution score: {bestDistribution.Score:F2}");

            // Correctness checks
            Console.WriteLine("\nCORRECTNESS CHECK:");
            Console.WriteLine(new string('=', 30));
            PerformValidationChecks(allPoints, robots, posts);
        }

        static void PerformValidationChecks(List<WeldingPoint> allPoints, List<Robot> robots, List<Post> posts)
        {
            bool allValid = true;
            List<string> errors = new List<string>();
            List<string> warnings = new List<string>();

            // 1. Check: all points are distributed
            var allAssignedPoints = robots.SelectMany(r => r.AssignedPoints).ToList();
            var uniqueAssigned = allAssignedPoints.Select(p => p.Name).Distinct().ToList();

            if (uniqueAssigned.Count != allPoints.Count)
            {
                errors.Add($"Distributed {uniqueAssigned.Count} out of {allPoints.Count} points");
                allValid = false;
            }
            else
            {
                Console.WriteLine($"✓ All {allPoints.Count} points distributed");
            }

            // 2. Check: each point assigned to only one robot
            if (allAssignedPoints.Count != uniqueAssigned.Count)
            {
                errors.Add("Some points assigned to more than one robot");
                allValid = false;
            }
            else
            {
                Console.WriteLine("✓ Each point assigned to exactly one robot");
            }

            // 3. Check: all robots are utilized
            var inactiveRobots = robots.Where(r => r.AssignedPoints.Count == 0).ToList();
            if (inactiveRobots.Any())
            {
                errors.Add($"Robots not utilized: {string.Join(", ", inactiveRobots.Select(r => r.Name))}");
                allValid = false;
            }
            else
            {
                Console.WriteLine("✓ All robots utilized");
            }

            // 4. Check: robots work on their own sides
            bool sideValid = true;
            foreach (var robot in robots)
            {
                var wrongSidePoints = robot.AssignedPoints.Where(p => p.Side != robot.Side).ToList();
                if (wrongSidePoints.Any())
                {
                    errors.Add($"Robot {robot.Name} ({robot.Side} side) received points from {wrongSidePoints.First().Side} side");
                    sideValid = false;
                    allValid = false;
                }
            }
            if (sideValid) Console.WriteLine("✓ Robots work only on their own sides");

            // 5. Check: gun type compatibility
            bool compatibilityValid = true;
            foreach (var robot in robots)
            {
                var incompatiblePoints = robot.AssignedPoints
                    .Where(p => !p.CompatibleGunTypes.Contains(robot.GunType))
                    .ToList();

                if (incompatiblePoints.Any())
                {
                    errors.Add($"Robot {robot.Name} ({robot.GunType}) received incompatible points: {string.Join(", ", incompatiblePoints.Take(3).Select(p => p.Name))}");
                    compatibilityValid = false;
                    allValid = false;
                }
            }
            if (compatibilityValid) Console.WriteLine("✓ All points compatible with robot gun types");

            // 6. Check load balancing between posts
            Console.WriteLine("\nLOAD BALANCING:");

            var postStats = new List<string>();
            foreach (var post in posts.OrderBy(p => p.Number))
            {
                int leftPoints = post.LeftRobots.Sum(r => r.AssignedPoints.Count);
                int rightPoints = post.RightRobots.Sum(r => r.AssignedPoints.Count);
                int totalPoints = leftPoints + rightPoints;

                postStats.Add($"Post {post.Number}: {totalPoints} points ({leftPoints}L/{rightPoints}R)");
            }

            Console.WriteLine($"  {string.Join(", ", postStats)}");

            var postTotals = posts.Select(p =>
                p.LeftRobots.Sum(r => r.AssignedPoints.Count) +
                p.RightRobots.Sum(r => r.AssignedPoints.Count)).ToList();

            double avgPoints = postTotals.Average();
            double imbalance = postTotals.Sum(p => Math.Abs(p - avgPoints));

            if (imbalance > 5)
                warnings.Add($"Uneven point distribution between posts (difference: {imbalance:F1} points)");
            else
                Console.WriteLine("✓ Load evenly distributed between posts");

            // 7. Check time balancing
            var postTimes = posts.Select(p => p.PostCycleTime).ToList();
            double maxTime = postTimes.Max();
            double minTime = postTimes.Min();
            double timeImbalance = maxTime - minTime;

            Console.WriteLine($"\nTIME BALANCING:");
            Console.WriteLine($"  Post times: {string.Join(" sec, ", postTimes.Select(t => t.ToString("F2"))) + " sec"}");
            Console.WriteLine($"  Maximum difference: {timeImbalance:F2} sec");

            if (timeImbalance > 5.0)
                warnings.Add($"Uneven post working time (difference: {timeImbalance:F2} sec)");
            else
                Console.WriteLine("✓ Post working time balanced");

            // 8. Check distribution by robots
            Console.WriteLine($"\nDISTRIBUTION BY ROBOTS:");
            var robotLoads = robots.Select(r => new {
                Robot = r.Name,
                Points = r.AssignedPoints.Count,
                Target = 10 // target points per robot
            }).ToList();

            foreach (var load in robotLoads)
            {
                Console.WriteLine($"  {load.Robot}: {load.Points} points (target: {load.Target})");
            }

            var perfectRobots = robotLoads.Count(r => r.Points == 10);
            if (perfectRobots == robots.Count)
                Console.WriteLine($"✓ All robots have optimal load (10 points)");
            else if (perfectRobots >= robots.Count * 0.8)
                Console.WriteLine($"✓ Most robots have optimal load");
            else
                warnings.Add($"Only {perfectRobots} out of {robots.Count} robots have optimal load");

            // Display warnings
            if (warnings.Any())
            {
                Console.WriteLine("\nWARNINGS:");
                foreach (var warning in warnings)
                    Console.WriteLine($"  ⚠ {warning}");
            }

            // Display errors
            if (errors.Any())
            {
                Console.WriteLine("\nERRORS:");
                foreach (var error in errors)
                    Console.WriteLine($"  ✗ {error}");
                allValid = false;
            }

            if (allValid && !warnings.Any())
                Console.WriteLine("\n✓ DISTRIBUTION IS CORRECT AND OPTIMAL!");
            else if (allValid)
                Console.WriteLine("\n✓ DISTRIBUTION IS CORRECT (has minor warnings)");
            else
                Console.WriteLine("\n✗ DISTRIBUTION HAS ERRORS!");
        }
    }
}