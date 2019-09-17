using Main;
using UnityEngine;
using UnityEngine.AI;
using System.Collections;
using System.Collections.Generic;

namespace WXR
{
    class MyTank : Tank
    {
        class Knowledge
        {
            public Tank enemy;

            public List<Vector3> myRoute;
            public List<Vector3> hisRoute;

            public NavMeshAgent myAgent;
            public NavMeshAgent hisAgent;

            public Vector3 turretOffset;
            public float turretLength;
            public float tankSpeed;
            public float tankAngularSpeed;
            public float missileSpeed;
        }

        private Knowledge knowledge;

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
            knowledge.missileSpeed = Match.instance.GlobalSetting.MissileSpeed;
        }

        private float m_LastTime = 0;

        bool MyCanSeeOthers(Vector3 target)
        {
            return true;
        }

        private void Shoot()
        {
            bool findTarget = false;
            Vector3 target = knowledge.enemy.Position;

            knowledge.hisRoute = new List<Vector3>(knowledge.hisAgent.path.corners);
            int iteration = 10;
            for (int i = 0; i < iteration; i++)
            {
                findTarget = false;
                float timeReach = ((target - Position).magnitude - knowledge.turretLength) / knowledge.missileSpeed;
                float timeSimulation = 0;
                Vector3 lastPosition = knowledge.enemy.Position;
                Vector3 lastForward = knowledge.enemy.transform.forward;
                foreach (var p in knowledge.hisRoute)
                {
                    Vector3 currentForward = p - lastPosition;
                    float timeTurn = Mathf.Abs(Vector3.Angle(currentForward, lastForward)) / knowledge.tankAngularSpeed;
                    timeSimulation += timeTurn;
                    if (timeSimulation > timeReach)
                    {
                        target = p;
                        findTarget = true;
                        break;
                    }

                    float timeRun = currentForward.magnitude / knowledge.tankSpeed;
                    if (timeSimulation + timeRun > timeReach)
                    {
                        target = Vector3.Lerp(lastPosition, p, (timeReach - timeSimulation) / timeRun);
                        findTarget = true;
                        break;
                    }
                    timeSimulation += timeRun;

                    lastPosition = p;
                    lastForward = currentForward;
                }
            }

            if (!findTarget)
            {
                findTarget = true;
                target = knowledge.enemy.Position;
            }

            if (findTarget)
            {
                TurretTurnTo(target);
                if (CanFire() && MyCanSeeOthers(target))
                {
                    Fire();
                }
            }

        }

        protected override void OnUpdate()
        {
            base.OnUpdate();

            Shoot();

            if (HP <= 50)
            {
                Move(Match.instance.GetRebornPos(Team));
            }
            else
            {
                bool hasStar = false;
                float nearestDist = float.MaxValue;
                Vector3 nearestStarPos = Vector3.zero;
                foreach (var pair in Match.instance.GetStars())
                {
                    Star s = pair.Value;
                    if (s.IsSuperStar)
                    {
                        hasStar = true;
                        nearestStarPos = s.Position;
                        break;
                    }
                    else
                    {
                        float dist = (s.Position - Position).sqrMagnitude;
                        if (dist < nearestDist)
                        {
                            hasStar = true;
                            nearestDist = dist;
                            nearestStarPos = s.Position;
                        }
                    }
                }
                if (hasStar == true)
                {
                    Move(nearestStarPos);
                }
                else
                {
                    if (Time.time > m_LastTime)
                    {
                        if (ApproachNextDestination())
                        {
                            m_LastTime = Time.time + Random.Range(3, 8);
                        }
                    }
                }
            }

            //Tank oppTank = Match.instance.GetOppositeTank(Team);
            //if (oppTank != null)
            //{
            //    if (CanSeeOthers(oppTank))
            //    {
            //        TurretTurnTo(oppTank.Position);
            //        Vector3 toTarget = oppTank.Position - FirePos;
            //        toTarget.y = 0;
            //        toTarget.Normalize();
            //        if (Vector3.Dot(TurretAiming, toTarget) > 0.98f)
            //        {
            //            Fire();
            //        }
            //    }
            //    else
            //    {
            //        TurretTurnTo(Position + Forward);
            //    }
            //}
        }
        protected override void OnReborn()
        {
            base.OnReborn();
            m_LastTime = 0;
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
            base.OnOnDrawGizmos();

            knowledge.myRoute = new List<Vector3>(knowledge.myAgent.path.corners);
            knowledge.hisRoute = new List<Vector3>(knowledge.hisAgent.path.corners);

            Gizmos.color = Match.instance.GetTeamColor(Team);
            Vector3 lastPoint = Position;
            foreach (var p in knowledge.myRoute)
            {
                Gizmos.DrawLine(lastPoint, p);
                lastPoint = p;
            }

            Gizmos.color = Match.instance.GetTeamColor(knowledge.enemy.Team);
            lastPoint = knowledge.enemy.Position;
            foreach (var p in knowledge.hisRoute)
            {
                Gizmos.DrawLine(lastPoint, p);
                lastPoint = p;
            }
        }
    }
}
