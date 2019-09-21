using Main;
using UnityEngine;
using UnityEngine.AI;
using System.Collections;
using System.Collections.Generic;

namespace WXR
{
    class MyTank : Tank
    {
        [System.Serializable]
        public class Knowledge
        {
            public Tank enemy;

            public List<Vector3> myRoute;
            public List<Vector3> hisRoute;

            public List<Vector3> hisExpectedRoute;

            public NavMeshAgent myAgent;
            public NavMeshAgent hisAgent;

            public Vector3 myRebornPos;
            public Vector3 hisRebornPos;

            public Vector3 myLastPosition;
            public Vector3 hisLastPosition;
            public Vector3 myLastShoot;

            public float hisLastVelocity;

            public Vector3 turretOffset;
            public float turretLength;
            public float tankSpeed;
            public float tankAngularSpeed;
            public float tankAcceleration;
            public float missileSpeed;
            public int damagePerHit;

            public float expectDistance;
        }

        public Knowledge knowledge;

        protected override void OnStart()
        {
            base.OnStart();
            knowledge = new Knowledge();
            knowledge.turretOffset = FirePos - Position;
            knowledge.turretLength = new Vector2(knowledge.turretOffset.x, knowledge.turretOffset.y).magnitude;
            knowledge.enemy = Match.instance.GetOppositeTank(Team);
            knowledge.myAgent = GetComponentInParent<NavMeshAgent>();
            knowledge.hisAgent = knowledge.enemy.GetComponentInParent<NavMeshAgent>();
            knowledge.tankSpeed = knowledge.myAgent.speed;
            knowledge.tankAngularSpeed = knowledge.myAgent.angularSpeed;
            knowledge.tankAcceleration = knowledge.myAgent.acceleration;
            knowledge.missileSpeed = Match.instance.GlobalSetting.MissileSpeed;
            knowledge.myRebornPos = Match.instance.GetRebornPos(Team);
            knowledge.hisRebornPos = Match.instance.GetRebornPos(knowledge.enemy.Team);
            knowledge.expectDistance = 10.0f;
            knowledge.myLastPosition = knowledge.myRebornPos;
            knowledge.hisLastPosition = knowledge.hisRebornPos;
            knowledge.myLastShoot = Vector3.zero;
            knowledge.hisLastVelocity = 0;
            knowledge.hisExpectedRoute = new List<Vector3>();
            knowledge.myRoute = new List<Vector3>(knowledge.myAgent.path.corners);
            knowledge.hisRoute = new List<Vector3>(knowledge.hisAgent.path.corners);
            knowledge.damagePerHit = Match.instance.GlobalSetting.DamagePerHit;
        }

        private bool MyCanSeeOthers(Vector3 target)
        {
            Vector3 origin = Position + (target - Position).normalized * knowledge.turretLength;
            bool result = !Physics.Linecast(target, origin, PhysicsUtils.LayerMaskScene);
            //if (result)
            //    Debug.DrawLine(origin, target, Color.magenta);
            //else
            //    Debug.DrawLine(origin, target, Color.black);
            //Debug.Log("origin: " + origin.ToString() + " target: " + target.ToString());
            return result;
        }

        private void Shoot()
        {
            bool findTarget = false;
            Vector3 target = knowledge.enemy.Position;
            float distance = (target - Position).magnitude;

            if (knowledge.enemy.IsDead)
            {
                knowledge.hisLastPosition = knowledge.hisRebornPos;
                target = knowledge.hisRebornPos;
                findTarget = true;
            }

            else if (distance < knowledge.expectDistance)
            {
                target = knowledge.enemy.Position;
                findTarget = true;
            }

            else
            {
                int iteration = 10;
                Vector3 lastPosition = knowledge.enemy.Position;
                Vector3 lastSpeed = (lastPosition - knowledge.hisLastPosition) / Time.deltaTime;
                Debug.DrawRay(lastPosition, lastSpeed * 2, Color.magenta);
                knowledge.hisLastVelocity = lastSpeed.magnitude;

                for (int i = 0; i < iteration; i++)
                {
                    findTarget = false;
                    float timeReach = ((target - Position).magnitude - knowledge.turretLength) / knowledge.missileSpeed;
                    float timeSimulation = 0;
                    const float timeSlice = 0.01f;
                    const float minSpeed = 0.01f;
                    lastPosition = knowledge.enemy.Position;
                    lastSpeed = (lastPosition - knowledge.hisLastPosition) / Time.deltaTime;
                    knowledge.hisExpectedRoute.Clear();
                    knowledge.hisExpectedRoute.Add(lastPosition);
                    foreach (var p in knowledge.hisRoute)
                    {
                        while ((lastPosition - p).sqrMagnitude > 1)
                        {
                            Vector3 toForward = p - lastPosition;

                            if (lastSpeed.magnitude < minSpeed)
                            {
                                lastSpeed = minSpeed * toForward.normalized;
                            }

                            float angle = Vector3.Angle(lastSpeed, toForward);

                            if (Mathf.Abs(angle) < 90)
                            {
                                if (lastSpeed.magnitude < knowledge.tankSpeed)
                                {
                                    if (lastSpeed.magnitude + knowledge.tankAcceleration * timeSlice > knowledge.tankSpeed)
                                        lastSpeed = knowledge.tankSpeed * lastSpeed.normalized;
                                    else
                                        lastSpeed = (lastSpeed.magnitude + knowledge.tankAcceleration * timeSlice) * lastSpeed.normalized;
                                }
                            }
                            else
                            {
                                if (lastSpeed.magnitude - knowledge.tankAcceleration * timeSlice > minSpeed)
                                    lastSpeed = (lastSpeed.magnitude - knowledge.tankAcceleration * timeSlice) * lastSpeed.normalized;
                                else
                                    lastSpeed = minSpeed * lastSpeed.normalized;
                            }

                            float angleDelta = knowledge.tankAngularSpeed * Mathf.Deg2Rad * timeSlice;

                            lastSpeed = Vector3.RotateTowards(lastSpeed, toForward,
                                knowledge.tankAngularSpeed * Mathf.Deg2Rad * timeSlice, 0.0f);

                            lastPosition += lastSpeed * timeSlice;
                            knowledge.hisExpectedRoute.Add(lastPosition);
                            timeSimulation += timeSlice;

                            if (timeSimulation > timeReach)
                            {
                                target = lastPosition;
                                findTarget = true;
                                break;
                            }
                        }

                        if (findTarget)
                        {
                            break;
                        }
                    }
                }

                if (!findTarget)
                {
                    target = lastPosition;
                    findTarget = true;
                }
            }

            if (!findTarget)
            {
                target = knowledge.enemy.Position;
                findTarget = true;
            }

            if (findTarget)
            {
                TurretTurnTo(target);
                if (MyCanSeeOthers(target) && CanFire())
                {
                    Fire();
                    knowledge.myLastShoot = target;
                }
            }
        }

        private float CalculateTime(Vector3 lastPosition, Vector3 origin, List<Vector3> positions)
        {
            Vector3 lastSpeed = (origin - lastPosition) / Time.deltaTime;
            lastPosition = origin;
            float timeSimulation = 0;
            const float timeSlice = 0.01f;
            const float minSpeed = 0.01f;
            foreach (var p in positions)
            {
                while ((lastPosition - p).sqrMagnitude > 1)
                {
                    Vector3 toForward = p - lastPosition;

                    if (lastSpeed.magnitude < minSpeed)
                    {
                        lastSpeed = minSpeed * toForward.normalized;
                    }

                    float angle = Vector3.Angle(lastSpeed, toForward);

                    if (Mathf.Abs(angle) < 90)
                    {
                        if (lastSpeed.magnitude < knowledge.tankSpeed)
                        {
                            if (lastSpeed.magnitude + knowledge.tankAcceleration * timeSlice > knowledge.tankSpeed)
                                lastSpeed = knowledge.tankSpeed * lastSpeed.normalized;
                            else
                                lastSpeed = (lastSpeed.magnitude + knowledge.tankAcceleration * timeSlice) * lastSpeed.normalized;
                        }
                    }
                    else
                    {
                        if (lastSpeed.magnitude - knowledge.tankAcceleration * timeSlice > minSpeed)
                            lastSpeed = (lastSpeed.magnitude - knowledge.tankAcceleration * timeSlice) * lastSpeed.normalized;
                        else
                            lastSpeed = minSpeed * lastSpeed.normalized;
                    }

                    float angleDelta = knowledge.tankAngularSpeed * Mathf.Deg2Rad * timeSlice;

                    lastSpeed = Vector3.RotateTowards(lastSpeed, toForward, angleDelta, 0.0f);
                    //Debug.DrawLine(lastPosition, lastPosition + lastSpeed * timeSlice, Color.white);

                    lastPosition += lastSpeed * timeSlice;
                    timeSimulation += timeSlice;
                }
            }

            return timeSimulation;
        }

        private float CalculateHisTime(Vector3 target)
        {
            if (knowledge.enemy.IsDead)
                return 100;

            if (knowledge.enemy.NextDestination == target)
                return CalculateTime(knowledge.hisLastPosition, knowledge.enemy.Position, knowledge.hisRoute);

            return CalculateTime(knowledge.hisLastPosition, knowledge.enemy.Position,
                new List<Vector3>(knowledge.enemy.CaculatePath(target).corners));
        }

        private float CalculateMyTime(Vector3 target)
        {
            if (NextDestination == target)
                return CalculateTime(knowledge.myLastPosition, Position, knowledge.myRoute);

            return CalculateTime(knowledge.myLastPosition, Position,
                new List<Vector3>(CaculatePath(target).corners));
        }

        private void Run()
        {
            bool hasStar = false;
            float nearestDist = float.MaxValue;
            Vector3 nearestStarPos = Vector3.zero;

            foreach (var pair in Match.instance.GetStars())
            {
                Star s = pair.Value;
                float dist = (s.Position - Position).sqrMagnitude;
                //float dist = CalculateMyTime(s.Position);
                if (s.IsSuperStar
                    && (knowledge.enemy.NextDestination != s.Position
                    || CalculateMyTime(s.Position) < CalculateHisTime(s.Position)))
                {
                    hasStar = true;
                    nearestDist = dist;
                    nearestStarPos = s.Position;
                    break;
                }
                else
                {
                    if (dist < nearestDist
                        && (knowledge.enemy.NextDestination != s.Position
                    || CalculateMyTime(s.Position) < CalculateHisTime(s.Position)))
                    {
                        hasStar = true;
                        nearestDist = dist;
                        nearestStarPos = s.Position;
                    }
                }
            }

            int hisShoot = (HP - 1) / knowledge.damagePerHit + 1;
            int myShoot = (knowledge.enemy.HP - 1) / knowledge.damagePerHit + 1;
            if (hisShoot < 2
                && (CalculateMyTime(knowledge.myRebornPos) * ((float)hisShoot / 2.0f)
                < CalculateMyTime(nearestStarPos)))
            {
                hasStar = false;
            }

            if (hasStar)
            {
                Move(nearestStarPos);
            }
            else if (!knowledge.enemy.IsDead && myShoot < hisShoot)
            {
                Move(knowledge.enemy.NextDestination);
            }
            else if (hisShoot < 4)
            {
                Move(knowledge.myRebornPos);
            }
            else if (!knowledge.enemy.IsDead)
            {
                Move(knowledge.enemy.NextDestination);
            }

            //Missile dangerousMissle = null;
            //float minTime = 0;
            //foreach (var m in Match.instance.GetOppositeMissiles(Team))
            //{
            //    Missile missle = m.Value;
            //    RaycastHit raycastHit;
            //    Physics.Raycast(missle.Position, missle.Velocity, out raycastHit, PhysicsUtils.LayerMaskScene);
            //    Vector3 misslePosition = missle.Position;
            //    Vector3 destination = raycastHit.point;

            //    bool hit = false;
            //    bool end = false;
            //    Vector3 lastSpeed = (Position - knowledge.myLastPosition) / Time.deltaTime;
            //    Vector3 lastPosition = Position;
            //    float timeSimulation = 0;
            //    const float timeSlice = 0.01f;
            //    const float minSpeed = 0.01f;
            //    foreach (var p in knowledge.myRoute)
            //    {
            //        if (hit || end)
            //        {
            //            break;
            //        }

            //        while ((lastPosition - p).sqrMagnitude > 1)
            //        {
            //            Vector3 toForward = p - lastPosition;

            //            if (lastSpeed.magnitude < minSpeed)
            //            {
            //                lastSpeed = minSpeed * toForward.normalized;
            //            }

            //            float angle = Vector3.Angle(lastSpeed, toForward);

            //            if (Mathf.Abs(angle) < 90)
            //            {
            //                if (lastSpeed.magnitude < knowledge.tankSpeed)
            //                {
            //                    if (lastSpeed.magnitude + knowledge.tankAcceleration * timeSlice > knowledge.tankSpeed)
            //                        lastSpeed = knowledge.tankSpeed * lastSpeed.normalized;
            //                    else
            //                        lastSpeed = (lastSpeed.magnitude + knowledge.tankAcceleration * timeSlice) * lastSpeed.normalized;
            //                }
            //            }
            //            else
            //            {
            //                if (lastSpeed.magnitude - knowledge.tankAcceleration * timeSlice > minSpeed)
            //                    lastSpeed = (lastSpeed.magnitude - knowledge.tankAcceleration * timeSlice) * lastSpeed.normalized;
            //                else
            //                    lastSpeed = minSpeed * lastSpeed.normalized;
            //            }

            //            float angleDelta = knowledge.tankAngularSpeed * Mathf.Deg2Rad * timeSlice;

            //            lastSpeed = Vector3.RotateTowards(lastSpeed, toForward, angleDelta, 0.0f);

            //            lastPosition += lastSpeed * timeSlice;
            //            misslePosition += missle.Velocity * timeSlice;
            //            timeSimulation += timeSlice;

            //            if ((misslePosition - destination).sqrMagnitude < 1)
            //            {
            //                end = true;
            //                break;
            //            }

            //            if ((misslePosition - lastPosition).sqrMagnitude < 1)
            //            {
            //                hit = true;
            //                break;
            //            }
            //        }
            //    }

            //    if (hit && timeSimulation > 1.0f && (dangerousMissle == null || timeSimulation < minTime))
            //    {
            //        dangerousMissle = missle;
            //        minTime = timeSimulation;
            //    }
            //}

            //Vector3 bestPosition = Vector3.zero;
            //if (dangerousMissle != null)
            //{
            //    Vector3[] positionOffsets = { new Vector3(-1, 0, -1), new Vector3(-1, 0, 0), new Vector3(-1, 0, -1),
            //    new Vector3(0, 0, -1), new Vector3(0, 0, 0), new Vector3(0, 0, -1),
            //    new Vector3(1, 0, -1), new Vector3(1, 0, 0), new Vector3(1, 0, -1)};

            //    Vector3 misslePosition = dangerousMissle.Position;
            //    Vector3 lastSpeed = (Position - knowledge.myLastPosition) / Time.deltaTime;
            //    Vector3 lastPosition = Position;
            //    float timeSimulation = 0;
            //    const float timeSlice = 0.01f;
            //    const float minSpeed = 0.01f;
            //    bool hit = false;
            //    bool end = false;

            //    for (int i = 0; i < positionOffsets.Length; i++)
            //    {
            //        positionOffsets[i] *= minTime * knowledge.tankSpeed;
            //        positionOffsets[i] += Position;

            //        foreach (var p in knowledge.myRoute)
            //        {
            //            if (hit || end)
            //            {
            //                break;
            //            }

            //            while ((lastPosition - p).sqrMagnitude > 1)
            //            {
            //                Vector3 toForward = p - lastPosition;

            //                if (lastSpeed.magnitude < minSpeed)
            //                {
            //                    lastSpeed = minSpeed * toForward.normalized;
            //                }

            //                float angle = Vector3.Angle(lastSpeed, toForward);

            //                if (Mathf.Abs(angle) < 90)
            //                {
            //                    if (lastSpeed.magnitude < knowledge.tankSpeed)
            //                    {
            //                        if (lastSpeed.magnitude + knowledge.tankAcceleration * timeSlice > knowledge.tankSpeed)
            //                            lastSpeed = knowledge.tankSpeed * lastSpeed.normalized;
            //                        else
            //                            lastSpeed = (lastSpeed.magnitude + knowledge.tankAcceleration * timeSlice) * lastSpeed.normalized;
            //                    }
            //                }
            //                else
            //                {
            //                    if (lastSpeed.magnitude - knowledge.tankAcceleration * timeSlice > minSpeed)
            //                        lastSpeed = (lastSpeed.magnitude - knowledge.tankAcceleration * timeSlice) * lastSpeed.normalized;
            //                    else
            //                        lastSpeed = minSpeed * lastSpeed.normalized;
            //                }

            //                float angleDelta = knowledge.tankAngularSpeed * Mathf.Deg2Rad * timeSlice;

            //                lastSpeed = Vector3.RotateTowards(lastSpeed, toForward, angleDelta, 0.0f);

            //                lastPosition += lastSpeed * timeSlice;
            //                misslePosition += dangerousMissle.Velocity * timeSlice;
            //                timeSimulation += timeSlice;

            //                if ((misslePosition - destination).sqrMagnitude < 1)
            //                {
            //                    end = true;
            //                    break;
            //                }

            //                if ((misslePosition - lastPosition).sqrMagnitude < 1)
            //                {
            //                    hit = true;
            //                    break;
            //                }
            //            }
            //        }
            //    }
            //}
        }

        protected override void OnUpdate()
        {
            //base.OnUpdate();

            knowledge.myRoute = new List<Vector3>(knowledge.myAgent.path.corners);
            knowledge.hisRoute = new List<Vector3>(knowledge.hisAgent.path.corners);

            Shoot();
            Run();

            Debug.DrawLine(knowledge.hisLastPosition, knowledge.enemy.Position, Color.black, 20.0f);
            knowledge.myLastPosition = Position;
            knowledge.hisLastPosition = knowledge.enemy.Position;
        }
        protected override void OnReborn()
        {
            base.OnReborn();
            knowledge.myLastPosition = knowledge.myRebornPos;
        }
        private bool ApproachNextDestination()
        {
            float halfSize = PhysicsUtils.MaxFieldSize * 0.5f;
            return Move(new Vector3(Random.Range(-halfSize, halfSize), 0, Random.Range(-halfSize, halfSize)));
        }
        public override string GetName()
        {
            return "WXR";
        }

        protected override void OnOnDrawGizmos()
        {
            //base.OnOnDrawGizmos();

            Gizmos.color = Match.instance.GetTeamColor(Team);
            Vector3 lastPoint = Position;
            foreach (var p in knowledge.myRoute)
            {
                Gizmos.DrawLine(lastPoint, p);
                lastPoint = p;
            }
            Gizmos.DrawWireSphere(Position, knowledge.expectDistance);
            Gizmos.DrawWireSphere(knowledge.myLastShoot, 2.0f);

            Gizmos.color = Match.instance.GetTeamColor(knowledge.enemy.Team);
            lastPoint = knowledge.enemy.Position;
            foreach (var p in knowledge.hisRoute)
            {
                Gizmos.DrawLine(lastPoint, p);
                lastPoint = p;
            }

            Gizmos.color = Color.yellow;
            lastPoint = knowledge.enemy.Position;
            foreach (var p in knowledge.hisExpectedRoute)
            {
                Gizmos.DrawLine(lastPoint, p);
                lastPoint = p;
            }
        }
    }
}
