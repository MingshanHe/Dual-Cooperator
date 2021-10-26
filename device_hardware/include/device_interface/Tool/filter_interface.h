/**
 * @file filter_interface.h
 *
 * @brief  滤波接口
 * @author chenchen
 * @version 1.0
 * @date 2020-09-11
 *
 */

#ifndef FILTER_INTERFACE_H_
#define FILTER_INTERFACE_H_



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

//-------------------------------滑动均值滤波----------------------
#define ONLINEFILTERNUM 10///< 滑动均值滤波器数组长度
#define MoveAverageFilter_BUF 2000///< 滑动均值滤波的缓冲区大小


/*-------------------------------------------------------------------------*/
/**
  @brief	滑动均值滤波

	滑动均值滤波器
 */
/*-------------------------------------------------------------------------*/
typedef struct MoveAverageFilter{
	int buf_num;///< 滑动均值滤波器数组长度
	double buf[MoveAverageFilter_BUF];///< 定义滑动均值滤波的缓冲区
	int _index;
	int one_flag;
}MoveAverageFilter;

/*-------------------------------------------------------------------------*/
/**
  @brief	滑动均值滤波

	滑动均值滤波器数组
 */
/*-------------------------------------------------------------------------*/
typedef struct MoveAverageFilters{
	MoveAverageFilter maf[ONLINEFILTERNUM];///< 滑动均值滤波器数组
	int dof;
}MoveAverageFilters;

/**
* @brief 创建在线滑动均值滤波器
*
* @param maf 滑动均值滤波器
* @param buf_num 缓存区数量
*/
extern void init_move_average_filter_online(MoveAverageFilter* maf, int buf_num);


extern double move_average_filter_online(MoveAverageFilter* maf, double value);

/**
* @brief 创建多个在线滑动均值滤波器
*
* @param maf 滑动均值滤波器数组对象
* @param buf_num 缓存区数量
*/
extern void init_move_average_filter_onlines(MoveAverageFilters* mafs, int buf_num, int _dof);


extern void move_average_filter_onlines(MoveAverageFilters* mafs, double* value_in, double* value_out, int _dof);



//-------------------------------------------------------------------卡尔曼滤波---------------------------------------------------------
#define MATRIX_MAX 100///< 矩阵最大维度

/*-------------------------------------------------------------------------*/
/**
  @brief	矩阵的结构体声明

	声明一个卡尔曼滤波器所需要用到的参数的矩阵结构定义
 */
/*-------------------------------------------------------------------------*/
typedef struct RMATRIX{
	double data[MATRIX_MAX][MATRIX_MAX]; ///< RMATRIX数组
	int row; ///< 行
	int col; ///< 列
}RMATRIX;

/*-------------------------------------------------------------------------*/
/**
  @brief	卡尔曼滤波

	卡尔曼滤波器
 */
/*-------------------------------------------------------------------------*/
typedef struct KalmanFilter{
	RMATRIX A,C,Q,R,P,K,I; 
	int m, n; ///< 矩阵尺寸
	double dt; 
	RMATRIX x_hat, x_hat_new;

	RMATRIX AT,CT;
	RMATRIX y;
	RMATRIX tmp, tmp1;///< 临时变量


}KalmanFilter;

/**
* @brief 使用指定的矩阵参数来创建卡尔曼滤波器
*
* @param dt 抽样时间
* @param A  系统动力学矩阵，尺寸为(n,n)
* @param C  输出矩阵，尺寸为(m,n)
* @param Q  过程噪声的协方差矩阵，尺寸为(n,n)
* @param R  测量噪声的协方差矩阵，尺寸为(m,m)
* @param P  估计误差的协方差矩阵，尺寸为(n,n)
* @param x0 初始状态，输入尺寸为(n,1)
*/
extern void initKalmanFilter(KalmanFilter* kf,double dt, RMATRIX* A, RMATRIX* C, RMATRIX* Q, RMATRIX* R, RMATRIX* P, RMATRIX* x0);


/**
* @brief 卡尔曼滤波器参数更新
*
* @param kf 创建的卡尔曼滤波器 
* @param y 测量值 用来修正参数
*/
extern RMATRIX* KalmanFilter_update(KalmanFilter* kf, RMATRIX* y);

/**
* @brief 卡尔曼滤波器参数更新
*
* @param kf 创建的卡尔曼滤波器
* @param y 测量值
* @param A
*/
extern RMATRIX* KalmanFilter_update1(KalmanFilter* kf, RMATRIX* y, RMATRIX* A);

/**
* @brief 获取卡尔曼滤波器的状态
*
* @param kf 创建的卡尔曼滤波器
*/
extern RMATRIX* getKalmanFilter_state(KalmanFilter* kf);

/**
* @brief 获取卡尔曼滤波器的输出
*
* @param kf 创建的卡尔曼滤波器
*/
extern RMATRIX* getKalmanFilter_out(KalmanFilter* kf);

/**
* @brief 使用指定的矩阵参数来创建卡尔曼滤波器
*
* @param dt 抽样时间
* @param A  系统动力学矩阵，尺寸为(n,n)
* @param C  输出矩阵，尺寸为(m,n)
* @param Q  过程噪声的协方差矩阵，尺寸为(n,n)
* @param R  测量噪声的协方差矩阵，尺寸为(m,m)
* @param P  估计误差的协方差矩阵，尺寸为(n,n)
* @param x0 初始状态，输入尺寸为(n,1)
* @param m 输入矩阵尺寸
* @param n 输入矩阵尺寸
*/
extern void initKalmanFilter_d(KalmanFilter* kf,double dt, double* A, double* C, double* Q, double* R, double* P, double* x0, int m, int n);

/**
* @brief 使用指定的矩阵参数来创建卡尔曼滤波器
*
* @param dt 抽样时间
* @param A  系统动力学矩阵，尺寸为(n,n)
* @param C  输出矩阵，尺寸为(m,n)
* @param Q  过程噪声的协方差矩阵，尺寸为(n,n)
* @param R  测量噪声的协方差矩阵，尺寸为(m,m)
* @param P  估计误差的协方差矩阵，尺寸为(n,n)
* @param x0 初始状态，输入尺寸为(n,1)
* @param m 输入矩阵尺寸
* @param n 输入矩阵尺寸
*/
extern void initKalmanFilter_d1(KalmanFilter* kf,double dt, double** A, double** C, double** Q, double** R, double** P, double* x0, int m, int n);

//return state(n,1)
extern double* KalmanFilter_update_d(KalmanFilter* kf, double* y, double* filter_state);

//return state(n,1)
extern double* KalmanFilter_update1_d(KalmanFilter* kf, double* y, double* A, double* filter_state);

//return state(n,1)
extern double* KalmanFilter_update1_d1(KalmanFilter* kf, double* y, double** A, double* filter_state);

//return state(n,1)
extern double* getKalmanFilter_state_d(KalmanFilter* kf, double* state);

//return out (m,1)
extern double* getKalmanFilter_out_d(KalmanFilter* kf, double* out);


//----------------------------------------------------数据滤波系数矩阵, 状态为位置，速度，加速度组成的向量-------------------------------------------------
extern double* getKalmanFilterA(double* A, double dt, int n);

extern double* getKalmanFilterC(double* C, double dt, int m);

extern double* getKalmanFilterQ(double* Q, double value, int n);

extern double* getKalmanFilterR(double* R, double value, int m);

extern double* getKalmanFilterP(double* P, double value, int n);


//-------------------------------------------------------------------IIR滤波---------------------------------------------------------
#define ARRAY_DIM 50  ///< 这个参数必须大于等于2*MAX_POLE_COUNT，因为一些滤波器的多项式使用2*NumPoles进行定义
/*-------------------------------------------------------------------------*/
/**
  @brief	多种无限脉冲响应滤波器

	TIIR infinite impulse response
 */
/*-------------------------------------------------------------------------*/
typedef enum TIIRPassTypes {
	iirLPF,  ///< 无限脉冲响应低通滤波
	iirHPF,  ///< 无限脉冲响应高通滤波
	iirBPF,  ///< 无限脉冲响应带通滤波
	iirNOTCH,  ///< 无限脉冲响应陷波滤波
	iirALLPASS ///< 无限脉冲响应全通滤波
}TIIRPassTypes;


typedef struct TIIRCoeff {
	double a0[ARRAY_DIM]; double a1[ARRAY_DIM]; double a2[ARRAY_DIM]; double a3[ARRAY_DIM]; double a4[ARRAY_DIM];
	double b0[ARRAY_DIM]; double b1[ARRAY_DIM]; double b2[ARRAY_DIM]; double b3[ARRAY_DIM]; double b4[ARRAY_DIM];
	int NumSections;
}TIIRCoeff;

/*-------------------------------------------------------------------------*/
/**
  @brief	一些可用的滤波器

	可用滤波器多项式
 */
/*-------------------------------------------------------------------------*/
typedef enum TFilterPoly {
	BUTTERWORTH,  ///< 低通滤波
	GAUSSIAN,  ///< 低通滤波
	BESSEL,  ///< 
	ADJUSTABLE,  ///< 
	CHEBYSHEV, ///< 
	INVERSE_CHEBY, ///< 
	PAPOULIS,  ///< 
	ELLIPTIC,  ///< 
	NOT_IIR ///< 测试用
}TFilterPoly;

/*-------------------------------------------------------------------------*/
/**
  @brief	TIIRF的参数定义

	需要定义滤波类型（高通、低通等等）、截止频率、带宽、增益、滤波器原型、极点数量、通带波纹、阻带衰减、过渡带宽调整系数
 */
/*-------------------------------------------------------------------------*/
typedef struct TIIRFilterParams {
	TIIRPassTypes IIRPassType;     ///< 定义滤波类型：低通，高通，等等
	double OmegaC;                 ///< 截止频率。无限脉冲响应滤波器的3dB角频率用于低通和高通滤波，中心频率用于带通和陷波滤波
	double BW;                     ///< 带宽。无限脉冲响应滤波器的3dB带宽用于带通和陷波滤波
	double dBGain;                 ///< 滤波器的增益

	///< 定义了要使用的低通滤波原型
	TFilterPoly ProtoType;  ///< 如Butterworth, Cheby等等
	int NumPoles;           ///< 极点数量*/
	double Ripple;          ///< 椭圆滤波和切比雪夫滤波的通带波纹
	double StopBanddB;      ///< 椭圆滤波和逆切比雪夫的阻带衰减（db）
	double Gamma;           ///< 可调整高斯滤波的过渡带宽系数，-1 <= Gamma <= 1 
}TIIRFilterParams;


/*-------------------------------------------------------------------------*/
/**
  @brief	TIIRF定义

	需要定义滤波类型（高通、低通等等）、截止频率、带宽、增益、滤波器原型、极点数量、通带波纹、阻带衰减、过渡带宽调整系数
 */
/*-------------------------------------------------------------------------*/
typedef struct TIIRFilter{
	TIIRFilterParams IIRFilt;  ///< 在IIRFilterCode.h中进行了定义
	TIIRCoeff IIRCoeff;
	double RegX1[ARRAY_DIM], RegX2[ARRAY_DIM], RegY1[ARRAY_DIM], RegY2[ARRAY_DIM];
	int flag_one;
}TIIRFilter;

/*-------------------------------------------------------------------------*/
/**
  @brief	TIIRF数组

	数组最大容量为是个TIIRF
 */
/*-------------------------------------------------------------------------*/
typedef struct TIIRFilters{
	TIIRFilter irrfiler[10];  ///< IIRF数组
	int n;
}TIIRFilters;


extern void initTIIRFilter(TIIRFilter* iirfiler, TIIRPassTypes types, double cornerFreq, double sampleFreq);

extern double IIRFilter(TIIRFilter* iirfiler, double Signal);

extern double* IIRFilterZeroPhase(TIIRFilter* iirfiler, double* Signal_in, double* Signal_out, int n);

extern void initTIIRFilters(TIIRFilters* iirfilers, TIIRPassTypes types, double cornerFreq, double sampleFreq, int n);

extern void IIRFilters(TIIRFilters* iirfilers, double* in, double* out);




#ifdef __cplusplus
}
}
#endif


#endif /* FILTER_INTERFACE_H_ */
