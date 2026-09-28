#include <BurTrajPlan/IBurTrajPlan.h>
#include "BurTrajPlan.h"
#include <BurTrajPlan/BurTrajPlan_global.h>
#include <boost/make_shared.hpp>
// IBurTrajPlan 是一个纯虚接口类，这里提供一些基本的实现
// 主要用于解决链接问题

namespace ModelTrajPlan
{
    // 纯虚析构函数的实现
    IBurTrajPlan::~IBurTrajPlan() {
        // 纯虚接口类的析构函数实现
        // 确保派生类能够正确析构
    }

    // === 工厂方法实现 - 按照PCP模式 ===
    boost::shared_ptr<IBurTrajPlan> IBurTrajPlan::create_trajplan()
    {
        return boost::make_shared<BurTrajPlan>();
    }

}