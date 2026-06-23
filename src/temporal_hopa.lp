% ============================================================
% Temporal-HOPA: 动态3D空间混合方向-距离-时间约束推理
% ASP Core Encoding (Clingo 5.8+)
% ============================================================

% ---- 域定义 ----
% 3D网格参数: grid(s,t,u), angular_resolution(R)
% s: X维度格点数, t: Y维度格点数, u: Z维度格点数
% R: 角度分辨率(度), 如 R=5 表示5度一格

#const s=50.  #const t=50.  #const u=20.
#const ang_res=5.  % 角度分辨率
#const max_time=5.  % 最大时间步

% 角度域: 0到359度, 步长为ang_res
angle(0..359, ang_res).

% 时间步
time(0..max_time).

% ---- 对象表示 ----
% 输入: object(K). point(K,P). (P是K的位置标识符)
% 如果对象位置未知, P=K

% ---- 生成候选空间配置 (每个时间步) ----
% X坐标
{xloc(P,T,X) : X=1..s} = 1 :- time(T), point(P), not loc_known(P,T).
% Y坐标
{yloc(P,T,Y) : Y=1..t} = 1 :- time(T), point(P), not loc_known(P,T).
% Z坐标
{zloc(P,T,Z) : Z=1..u} = 1 :- time(T), point(P), not loc_known(P,T).

% 朝向角 (heading: 0-359)
{heading(K,T,H) : angle(H,ang_res)} = 1 :- time(T), object(K), not orient_known(K,T).

% 俯仰角 (pitch: -90..90, 步长ang_res)
pitch_val(-90..90, ang_res).
{pitch(K,T,Pv) : pitch_val(Pv,ang_res)} = 1 :- time(T), object(K), not pitch_known(K,T).

% ---- 空间约束编码 ----
% 3D方向约束 (27方向CDC: 在XY平面用8方向, Z轴用3层)
% XY平面方向: N=0, NE=1, E=2, SE=3, S=4, SW=5, W=6, NW=7
% Z层: below=0, same=1, above=2

% 方向判定规则 (对象K相对于L的3D方向)
% XY方向基于反正切区间
xy_dir(P,R,T,D_XY) :- xloc(P,T,X1), xloc(R,T,X2), yloc(P,T,Y1), yloc(R,T,Y2),
    X1!=X2, D_XY = (X2-X1, Y2-Y1),
    D_XY = @xy_direction(X1,Y1,X2,Y2).

% Z方向
z_dir(P,R,T,0) :- zloc(P,T,Z1), zloc(R,T,Z2), Z2 < Z1 - 1.  % below
z_dir(P,R,T,1) :- zloc(P,T,Z1), zloc(R,T,Z2), |Z2-Z1| <= 1. % same
z_dir(P,R,T,2) :- zloc(P,T,Z1), zloc(R,T,Z2), Z2 > Z1 + 1.  % above

% ---- 3D OPRA-like约束 ----
% 输入: t_opra_3d(N,K,L,XY_SECTOR,Z_LAYER,G,T) 
% N=约束编号, G=粒度, T=时间步
% XY_SECTOR取值为0..4G-1 (来自OPRA_m定义, m=G)
% Z_LAYER: -1=below, 0=same, 1=above

% 差分OPRA 3D: 需要同时满足XY扇区和Z层
% 检查: K在L的扇区XY_SECTOR内, 且Z关系匹配Z_LAYER
:- t_opra_3d(N,K,L,I,ZL,G,T), point(K,P), point(L,R),
   not opra_satisfied_3d(P,R,I,ZL,G,T).

% ---- 定性距离约束 ----
% 输入: t_dist_rel(N,K,L,REL,T)  其中REL: very_near, near, medium, far
% 输入: t_dist_range(N,K,L,D1,D2,T)  定量距离范围

% 距离区间定义 (可在输入中覆盖)
dist_interval(very_near, 0, 2).
dist_interval(near, 1, 5).
dist_interval(medium, 4, 12).
dist_interval(far, 10, 50).

% 计算3D欧氏距离的平方 (避免sqrt)
sq_dist_3d(P,R,T,SD) :- xloc(P,T,X1), xloc(R,T,X2), yloc(P,T,Y1), yloc(R,T,Y2),
    zloc(P,T,Z1), zloc(R,T,Z2),
    SD = (X1-X2)*(X1-X2) + (Y1-Y2)*(Y1-Y2) + (Z1-Z2)*(Z1-Z2).

% 定性距离约束检查
:- t_dist_rel(N,K,L,REL,T), point(K,P), point(L,R),
   dist_interval(REL, DL, DU),
   sq_dist_3d(P,R,T,SD), SD < DL*DL.
:- t_dist_rel(N,K,L,REL,T), point(K,P), point(L,R),
   dist_interval(REL, DL, DU),
   sq_dist_3d(P,R,T,SD), SD > DU*DU.

% 定量距离约束
:- t_dist_range(N,K,L,D1,D2,T), point(K,P), point(L,R),
   sq_dist_3d(P,R,T,SD), SD < D1*D1.
:- t_dist_range(N,K,L,D1,D2,T), point(K,P), point(L,R),
   sq_dist_3d(P,R,T,SD), SD > D2*D2.

% 精确距离约束
:- t_dist_precise(N,K,L,D,T), point(K,P), point(L,R),
   sq_dist_3d(P,R,T,SD), |SD - D*D| > 1.

% ---- 定量空间约束 ----
% 位置约束
:- t_location(N,K,X,Y,Z,T), point(K,P), xloc(P,T,X1), X1 != X.
:- t_location(N,K,X,Y,Z,T), point(K,P), yloc(P,T,Y1), Y1 != Y.
:- t_location(N,K,X,Y,Z,T), point(K,P), zloc(P,T,Z1), Z1 != Z.

% 朝向约束
:- t_heading(N,K,H,T), heading(K,T,H1), H1 != H.
:- t_heading_range(N,K,H1,H2,T), heading(K,T,H), H < H1.
:- t_heading_range(N,K,H1,H2,T), heading(K,T,H), H > H2.

% 俯仰角约束
:- t_pitch(N,K,Pv,T), pitch(K,T,Pv1), Pv1 != Pv.
:- t_pitch_range(N,K,P1,P2,T), pitch(K,T,Pv), Pv < P1.
:- t_pitch_range(N,K,P1,P2,T), pitch(K,T,Pv), Pv > P2.

% ---- 时间运动约束 (Temporal Motion) ----
% 运动连续性: 相邻时间步的位置变化受速度限制

% 速度计算: 对象K在时间步T到T+1之间的距离
move_dist_3d(K,T,MD) :- time(T), T < max_time, object(K), point(K,P),
    xloc(P,T,X1), xloc(P,T+1,X2),
    yloc(P,T,Y1), yloc(P,T+1,Y2),
    zloc(P,T,Z1), zloc(P,T+1,Z2),
    MD = (X1-X2)*(X1-X2) + (Y1-Y2)*(Y1-Y2) + (Z1-Z2)*(Z1-Z2).

% 最大速度约束 (单位: 格点/时间步)
% 输入: t_max_speed(K,Smax)  (Smax^2 用于比较)
:- t_max_speed(K,Smax), move_dist_3d(K,T,MD), MD > Smax*Smax.

% 最大转向率约束 (heading变化)
% 输入: t_max_turn_rate(K,Rmax)  (度/时间步)
:- time(T), T < max_time, object(K), heading(K,T,H1), heading(K,T+1,H2),
   t_max_turn_rate(K,Rmax),
   |H2 - H1| > Rmax, |H2 - H1| < 360 - Rmax.

% 注意: 360度环绕情况
:- time(T), T < max_time, object(K), heading(K,T,H1), heading(K,T+1,H2),
   t_max_turn_rate(K,Rmax),
   H1 > 360 - Rmax, H2 < Rmax, 360 - H1 + H2 > Rmax.
:- time(T), T < max_time, object(K), heading(K,T,H1), heading(K,T+1,H2),
   t_max_turn_rate(K,Rmax),
   H2 > 360 - Rmax, H1 < Rmax, 360 - H2 + H1 > Rmax.

% 最大加速度约束 (速度变化)
:- time(T), T+1 < max_time, object(K),
   move_dist_3d(K,T,MD1), move_dist_3d(K,T+1,MD2),
   t_max_accel(K,Amax),
   |MD2 - MD1| > Amax*Amax.

% ---- 析取约束 (不确定性) ----
% 输入: t_disj(N,D,TYPE,...) 
% {chosen(N,D) : disj_index(N,D)} = 1 :- disj_constraint(N).

% ---- 可废止/假设约束 ----
% 输入: t_presumed(描述) - 优先级低于强制约束
% 最小化违反的假设约束数

% ---- 不一致解释 ----
% 输入: t_mandatory(N). 标记不可丢弃的约束
% {drop(N)} <= 1 :- not t_mandatory(N), constraint_id(N).
% :~ drop(N). [1@1, N]

% ---- 显示输出 ----
#show xloc/3.
#show yloc/3.
#show zloc/3.
#show heading/3.
#show pitch/3.
#show move_dist_3d/2.
