#pragma once

#define _USE_MATH_DEFINES  // 蹇呴』鍦?<cmath> 涔嬪墠瀹氫箟
#include <cmath>           // 鍖呭惈 M_PI 瀹氫箟

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <SlicingTrajPlan/GeneralData_global.h>
#include "SlicingData.h"  // 鍖呭惈SlicingParams鐨勫畬鏁村畾涔?
#include <map>
namespace PublicData { enum class CoordinateFrame { World }; }  // 鍖呭惈 CoordinateFrame 鏋氫妇瀹氫箟

// 濡傛灉 M_PI 浠嶆湭瀹氫箟锛堟煇浜涚紪璇戝櫒锛夛紝鎵嬪姩瀹氫箟
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ModelTrajPlan
{
    // 銆愭柊澧炪€戣建杩瑰垎缁勪俊鎭紙鐢ㄤ簬杩斿洖杞ㄨ抗缁熻锛? 浠?TrajectoryGenerator.h 杩佺Щ
    struct TrajectoryGroupInfo {
        int numTrajectories = 0;       // 杞ㄨ抗鏉℃暟
        int pointsPerTrajectory = 0;   // 姣忔潯杞ㄨ抗鐨勭偣鏁?
        bool isConsistent = true;      // 鎵€鏈夎建杩圭偣鏁版槸鍚︿竴鑷?
        TrajectoryGroupInfo() = default;
    };



    // 銆愰噸鏋勬柊澧炪€戣建杩圭敓鎴愬弬鏁扮粨鏋?- 浠嶴licingParams鍒嗙鍑烘潵鐨勫亸绉诲弬鏁?
    struct TrajectoryGenerationParams {
        int normalAxis = 4;
        int forwardAxis = 0;
        bool previewFirstPoint = false;
        bool upperTransitionSmoothPose = false;
        bool lowerTransitionSmoothPose = false;
        // Direction selection is independent of transition-point orientation.
        bool upperTransitionOppositeOffset = false;
        bool lowerTransitionOppositeOffset = false;
        bool upperTransitionNormalOffset = true;
        bool lowerTransitionNormalOffset = true;
        bool upperTransitionFollowSprayDistance = true;
        bool lowerTransitionFollowSprayDistance = true;
        double upperTransitionSprayDistance = 50.0;
        double lowerTransitionSprayDistance = 50.0;
        // Physical upper/lower boundary points have their own signed spray
        // distance.  The transition-point settings above remain independent.
        bool upperBoundaryFollowGlobalSprayDistance = true;
        bool lowerBoundaryFollowGlobalSprayDistance = true;
        double upperBoundarySprayDistance = 0.0;
        double lowerBoundarySprayDistance = 0.0;
        // 鍋忕Щ鍙傛暟
        double z1Offset = 0.0;            // 璧峰Z鍋忕Щ锛堜笅杈圭晫鍋忕Щ锛?
        double z2Offset = 0.0;            // 缁撴潫Z鍋忕Щ锛堜笂杈圭晫鍋忕Щ锛?
        double offsetDistance = 50.0;      // 娉曠嚎鍋忕Щ璺濈
        // Optional per-slice distances. Empty means use offsetDistance.
        std::vector<double> layerSprayDistances;
        std::vector<bool> layerSprayDistanceFollowGlobal;

        // 閫熷害鍙傛暟
        double normalVelocity = 50.0;     // 姝ｅ父杞ㄨ抗閫熷害 (mm/s)
        double transitionVelocity = 50.0; // 杩囨浮娈甸€熷害 (mm/s)
        double acceleration = 0.0;        // 鍔犻€熷害 (mm/s)

        TrajectoryGenerationParams() = default;
        TrajectoryGenerationParams(double z1, double z2, double offset,
            double normalVel = 50.0, double transVel = 50.0,
            double accel = 0.0, double timeInt = 0.1)
            : z1Offset(z1), z2Offset(z2), offsetDistance(offset),
            normalVelocity(normalVel), transitionVelocity(transVel),
            acceleration(accel) {}
    };


    // 杩炴帴绛栫暐鏋氫妇
    enum class ConnectionStrategy {
        Sequential = 0,    // 椤哄簭杩炴帴
        Optimized = 1,     // 浼樺寲杩炴帴
        ZigZag = 2,        // Z瀛楀舰杩炴帴
        LayerByLayer = 3,  // 閫愬眰杩炴帴
        BowPath = 4        // 寮撳舰璺緞杩炴帴
    };

    // 銆愭柊澧炪€戣建杩硅繛鎺ュ弬鏁扮粨鏋?- 浠?TrajectoryConnector.h 杩佺Щ
    struct TrajectoryConnectionParams {
        int numTrajectories = 0;      // 杞ㄨ抗鏁伴噺
        int pointsPerTrajectory = 0;  // 姣忔潯杞ㄨ抗鐨勭偣鏁?
        ConnectionStrategy strategy = ConnectionStrategy::Sequential; // 杩炴帴绛栫暐

        TrajectoryConnectionParams() = default;
    };

    // 杞ㄨ抗缁撴瀯 - 鍖呭惈浣嶅Э鍜屾椂闂存埑
    struct TrajectoryPoint {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();  // 榻愭鐭╅樀濮挎€佷綅缃級
        double spray_distance = 0.0;                         // 鍠锋秱璺濈锛坢m锛夛紝宸ュ叿鍒板伐浠惰〃闈㈢殑璺濈
        // ====== Debug / indexing metadata (non-functional) ======
        // 鐢ㄤ簬鏍囪瘑鈥滅鍑犳潯杞ㄨ抗-绗嚑灞傗€濓紝渚嬪锛歵rajectory_index=0, layer_index=1 琛ㄧず "01"
        int trajectory_index = -1;                           // 杞ㄨ抗绱㈠紩锛坱rajectoryIndex锛?
        int layer_index = -1;                                // 灞傜储寮曪紙layerIndex锛?

        double timestamp = 0.0;                              // 鏃堕棿鎴?
        double velocity = 0.0;                               // 閫熷害
        double acceleration = 0.0;                           // 鍔犻€熷害
        int point_type = 0;                                  // 鐐圭被鍨嬶紙0=鏅€氱偣锛?=杈圭晫鐐癸紝2=鍋忕Щ杩囨浮鐐癸級
    };

    // CoordinateFrame 鏋氫妇瀹氫箟鍦?PublicData/TrajData/TrajData.h 涓?

    struct TolerancedTrajectoryPoint {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

        Eigen::Matrix<double, 3, 2> pos_tolerance;
        Eigen::Matrix<double, 3, 2> orient_tolerance;
        Eigen::Vector2d vel_range;//閫熺巼鑼冨洿

        bool is_adjustable;
        PublicData::CoordinateFrame tolerance_frame;
        double spray_distance = 0.0;  // 鍠锋秱璺濈锛坢m锛夛紝宸ュ叿鍒板伐浠惰〃闈㈢殑璺濈

        TolerancedTrajectoryPoint()
            : pos_tolerance(Eigen::Matrix<double, 3, 2>::Zero()),
            orient_tolerance(Eigen::Matrix<double, 3, 2>::Zero()),
            vel_range(Eigen::Vector2d::Zero()),
            is_adjustable(true),
            tolerance_frame(PublicData::CoordinateFrame::World),
            spray_distance(0.0)
        {
            pose = Eigen::Matrix4d::Identity();
        }
    };

    // 瀹瑰樊鎼滅储閰嶇疆缁撴瀯浣?
    struct ToleranceLimitConfig {
        // ========== 鎼滅储鑼冨洿 ==========
        Eigen::Vector3d max_translation;  // mm, 姣忎釜杞寸殑鏈€澶у钩绉绘悳绱㈣寖鍥? e.g. [50, 30, 10]
        double spray_distance_adjustment_range = 50.0;
        Eigen::Vector3d max_rotation;     // rad, 姣忎釜杞寸殑鏈€澶ф棆杞悳绱㈣寖鍥? e.g. [M_PI, M_PI/4, M_PI/3]

        // ========== 浜屽垎鎼滅储鍋滄鏉′欢 ==========
        double position_tolerance;        // mm, 浣嶇疆鎼滅储绮惧害锛堝仠姝㈡潯浠讹級锛岄粯璁?0.1mm
        double rotation_tolerance;        // rad, 鏃嬭浆鎼滅储绮惧害锛堝仠姝㈡潯浠讹級锛岄粯璁?0.5掳 = 0.00873 rad

        // ========== 鏋勯€犲嚱鏁帮紙璁剧疆榛樿鍊硷級 ==========
        ToleranceLimitConfig()
            : max_translation(50.0, 30.0, 10.0),
            max_rotation(M_PI, M_PI / 4.0, M_PI / 3.0),
            position_tolerance(0.1),           // 0.1mm
            rotation_tolerance(0.00873)        // 0.5掳 in radians
        {}
    };

    // 杩炴帴绾挎淇℃伅缁撴瀯 - 浠?TrajectoryConnector.h 绉诲姩
    struct ConnectionSegment {
        Eigen::Vector3d startPoint = Eigen::Vector3d::Zero();        // 璧峰鐐?
        Eigen::Vector3d endPoint = Eigen::Vector3d::Zero();          // 缁撴潫鐐?
        Eigen::Vector3f color = Eigen::Vector3f(1.0f, 1.0f, 1.0f);  // 绾挎棰滆壊
        float lineWidth = 2.0f;                                      // 绾垮
        int segmentType = 0;                                         // 绾挎绫诲瀷锛?=杞ㄨ抗鍐呰繛鎺ワ紝1=杞ㄨ抗闂磋繛鎺ワ紝2=杩囨浮杩炴帴
    };

    // 杩炴帴鏁版嵁缁撴瀯 - 瀛樺偍杞ㄨ抗杩炴帴鐨勫畬鏁寸粨鏋?
    struct ConnectionData {
        std::vector<TrajectoryPoint> connectedTrajectory_;           // 杩炴帴鍚庣殑瀹屾暣杞ㄨ抗
        std::vector<ConnectionSegment> connectionSegments_;          // 杩炴帴绾挎
        ConnectionStrategy strategy_ = ConnectionStrategy::Sequential; // 浣跨敤鐨勮繛鎺ョ瓥鐣?
        bool isConnected_ = false;                                   // 鏄惁宸茶繛鎺?

        // 杩炴帴鍙傛暟
        double transitionSpeed_ = 30.0;
        double normalSpeed_ = 50.0;
        double connectionDistance_ = 5.0;
    };

    // 绠€鍖栫殑鏁版嵁缁撴瀯瀹氫箟锛屾潵鑷?TrajectoryGenerator.h - 閲嶅懡鍚嶉伩鍏嶄笌ThicknessCalculator鍐茬獊
    struct TrajVector3D {
        double x = 0.0, y = 0.0, z = 0.0;
        TrajVector3D() = default;
        TrajVector3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    };

    struct Vector4D {
        double x = 0.0, y = 0.0, z = 0.0, w = 0.0;
        Vector4D() = default;
        Vector4D(double x_, double y_, double z_, double w_) : x(x_), y(y_), z(z_), w(w_) {}
    };


    class GeneralData_SHAREDLIB TrajData
    {
    public:
        TrajData();
        ~TrajData();

        // 鏁版嵁鎴愬憳 - 鍏紑璁块棶锛屼粎鏁版嵁瀹瑰櫒
        std::vector<TrajectoryPoint> trajectoryPoints_;        // 涓昏建杩圭偣闆嗗悎
        std::map<double, TolerancedTrajectoryPoint> trajectoryPointsbf_;
        std::map<double, TolerancedTrajectoryPoint> trajectoryPointsaf_;

        ConnectionData connectionData_;                        // 杩炴帴鏁版嵁锛堝畬鏁寸殑杩炴帴缁撴灉锛?
        ToleranceLimitConfig tol_limit;   //瀹瑰樊鑼冨洿

        // 銆愭柊澧炪€戦噰鏍风偣鏁版嵁 - 浠庤竟鐣岀嚎鍧囧寑閲囨牱寰楀埌鐨勭偣 [trajectoryIndex][layerIndex]
        std::vector<std::vector<Eigen::Vector3d>> sampledBoundaryPoints_;

        // 銆愰噸鏋勩€戣建杩瑰弬鏁扮鐞?- 浠庡悇涓畻娉曟ā鍧楄縼绉昏繃鏉?
        TrajectoryGenerationParams generationParams_;         // 杞ㄨ抗鐢熸垚鍙傛暟锛堝亸绉诲弬鏁帮級
        TrajectoryConnectionParams connectionParams_;          // 杞ㄨ抗杩炴帴鍙傛暟
        TrajectoryGroupInfo groupInfo_;                       // 杞ㄨ抗鍒嗙粍淇℃伅

    private:

    };

}



