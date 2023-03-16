from control.matlab import *
import numpy as np


## https://algorithm.joho.info/seigyoriron/python-control-optimal-regulator/

# 可制御性のチェック
def check_ctrb(A, B):
  Uc = ctrb(A, B) # 可制御性行列の計算
  Nu = np.linalg.matrix_rank(Uc)  # Ucのランクを計算
  (N, N) = np.matrix(A).shape     # 正方行列Aのサイズ(N*N)
  # 可制御性の判別
  if Nu == N: return 0            # 可制御
  else: return -1                 # 可制御でない

  # 可観測性のチェック
def check_obsv(A, C):
  Uo = obsv(C, A) # 可制御性行列の計算
  No = np.linalg.matrix_rank(Uo)  # Ucのランクを計算
  (N, N) = np.matrix(A).shape     # 正方行列Aのサイズ(N*N)
  # 可制御性の判別
  if No == N: return 0            # 可観測
  else: return -1                 # 可観測でない
    
def main():
  # parameter
  mc = 1 # 台車質量
  mp = 0.1 # 振子質量
  M = 4*mc + mp
  g = 9.8
  D = 0  # 台車摩擦係数
  l = 0.25 # 振子の重心までの長さ
  a11 = 0; a12 = 0; a13 = 1; a14 = 0
  a21 = 0; a22 = 0; a23 = 0; a24 = 1
  a31 = 0; 
  a32 = -3*mp*g/M; 
  a33 = -4*D/M; 
  a34 = 0
  a41 = 0;
  a42 = 3*(mc+mp)*g/(M*l);
  a43 = 3*D/(M*l);
  a44 = 0
  b11 = 0; b21 = 0; b31 = 4/M; b41 = -3/(M*l)

  # システム行列の定義
  A = np.array([[a11, a12, a13, a14],
                [a21, a22, a23, a24],
                [a31, a32, a33, a34],
                [a41, a42, a43, a44]])
  B = np.array([[b11],
                [b21],
                [b31],
                [b41]])
  Q = np.identity(4)
  R = np.array([[1]])
  # システムが可制御・可観測でなければ終了
  if check_ctrb(A, B) == -1 :
    print("システムが可制御でないので終了")
    return 0
  if check_obsv(np.sqrt(Q), A) == -1 :
    print("システムが可観測でないので終了")
    return 0
  # 最適レギュレータの設計
  K, P, e = lqr(A, B, Q, R)
  # 結果表示
  print("リカッチ方程式の解:\n",P)
  print("状態フィードバックゲイン:\n",K)
  print("閉ループ系の固有値:\n",e)

if __name__ == "__main__":
  main()