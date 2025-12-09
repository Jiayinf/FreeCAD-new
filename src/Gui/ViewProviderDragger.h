/***************************************************************************
 *   Copyright (c) 2017 Werner Mayer <wmayer[at]users.sourceforge.net>     *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/


#ifndef GUI_VIEWPROVIDER_DRAGGER_H
#define GUI_VIEWPROVIDER_DRAGGER_H

#include "ViewProviderDocumentObject.h"
#include <Inventor/nodes/SoAnnotation.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/nodes/SoText2.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoVertexProperty.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoDrawStyle.h>  // 用于设置线条样式（虚线、宽度）
#include <Inventor/nodes/SoSwitch.h>
#include <Gui/SoFCCSysDragger.h>
#include <Base/Vector3D.h>
#include <Base/Rotation.h>


class SoDragger;
class SoTransform;

namespace Base
{
class Placement;
}

namespace Gui
{

class View3DInventorViewer;
class SoFCCSysDragger;

/**
 * The base class for all view providers modifying the placement
 * of a geometric feature.
 * @author Werner Mayer
 */
class GuiExport ViewProviderDragger: public ViewProviderDocumentObject
{
    PROPERTY_HEADER_WITH_OVERRIDE(Gui::ViewProviderDragger);

public:
    /// constructor.
    ViewProviderDragger();

    /// destructor.
    ~ViewProviderDragger() override;

    /** @name Edit methods */
    //@{
    bool doubleClicked() override;
    void setupContextMenu(QMenu*, QObject*, const char*) override;
    void updateData(const App::Property*) override;

    ViewProvider* startEditing(int ModNum = 0) override;

    /*! synchronize From FC placement to Coin placement*/
    static void updateTransform(const Base::Placement& from, SoTransform* to);

    // 初始化临时辅助图形
    void initTempGeometry();
    // 清理临时辅助图形
    void cleanupTempGeometry();
    // 更新平移辅助图形（直线+距离标注）
    void updateTranslationGeometry(SoFCCSysDragger* draggerIn);
    // 更新旋转辅助图形（弧线+弧度标注）
    void updateRotationGeometry(const SbRotation& currentRot, const SbVec3f& center);
    // 辅助方法：控制组件显示/隐藏
    void setComponentVisible(SoSwitch* pSwitch, bool visible);
    // void attach(Gui::ViewProviderDocumentObject* parent) override;

    // 11.28 新增：用户输入距离功能
    // 返回当前黑线上记录的拖动距离（mm）
    double getCurrentDragDistance() const;

    // 根据用户输入的新距离，沿着当前方向移动物体
    void applyNumericTranslation(double newDistanceMM);


protected:
    bool setEdit(int ModNum) override;
    void unsetEdit(int ModNum) override;
    void setEditViewer(View3DInventorViewer*, int ModNum) override;
    void unsetEditViewer(View3DInventorViewer*) override;
    //@}
    SoFCCSysDragger* csysDragger = nullptr;

private:
    // --- 新增：辅助函数声明 ---
    // 获取拖拽器中心的世界坐标
    SbVec3f getDraggerWorldPosition(SoFCCSysDragger* dragger);

    // --- 新增：回调函数声明 ---
    static void dragStartCallback(void* data, SoDragger* dragger);
    static void dragMotionCallback(void* data, SoDragger* dragger);


    static void dragFinishCallback(void* data, SoDragger* d);
    static void updatePlacementFromDragger(ViewProviderDragger* sudoThis,
                                           SoFCCSysDragger* draggerIn);

    bool checkLink();

    ViewProvider* _linkDragger = nullptr;

    // 临时辅助图形相关节点
    SoAnnotation* m_pTempAnnotation = nullptr;  // 标注容器（统一管理临时图形）


    // 平移直线相关（添加 SoSwitch 控制可见性）
    SoSwitch* m_pTranslationSwitch = nullptr;  // 平移组件总开关
    SoLineSet* m_pTranslationLine = nullptr;   // 平移直线
    SoMaterial* m_pLineMat = nullptr;

    // 旋转圆弧相关（添加 SoSwitch 控制可见性）
    SoSwitch* m_pRotationSwitch = nullptr;       // 旋转组件总开关
    SoIndexedLineSet* m_pRotationArc = nullptr;  // 旋转弧线
    SoMaterial* m_pArcMat = nullptr;             // 圆弧材质单独管理
    SoDrawStyle* m_pArcDrawStyle = nullptr;      // 圆弧线条样式（虚线）

    // 标注文本相关（添加 SoSwitch 控制可见性）
    SoSwitch* m_pTextSwitch = nullptr;        // 文本组件总开关
    SoText2* m_pDimensionText = nullptr;      // 距离/弧度标注文本
    SoTransform* m_pTextTransform = nullptr;  // 文本位置变换（避免遮挡）
    SoMaterial* m_pTextMat = nullptr;

    // 11.27 新增：
    SbVec3f m_vStartPos;         // 拖动起始位置（全局坐标）
    SbRotation m_rStartRot;      // 拖动起始旋转（全局坐标）
    bool m_bIsDragging = false;  // 是否正在拖动

    // 关键：正确声明新增成员（类型+名称，初始化 nullptr/默认值）
    SbVec3f m_draggerStartTrans = SbVec3f(0.0f, 0.0f, 0.0f);  // 拖动器起始平移（默认零向量）
    SbRotation m_draggerStartRot = SbRotation();              // 拖动器起始旋转（默认单位矩阵）

    SbMatrix m_startLocalToWorldMat;    // 拖动开始时的局部→全局变换矩阵
    SbMatrix m_currentLocalToWorldMat;  // 拖动当前的局部→全局变换矩阵


    SbVec3f m_initialDraggerWorld;     // 存储初始拖动器位置
    bool m_hasInitialDragger = false;  // 标志位，确保只记录一次初始坐标

    // 同一轴方向累积的平移向量（世界坐标系下）
    SbVec3f transDelta;
    SbVec3f m_lastTranslationDelta;

    //// 累积的平移向量（世界坐标系下）
    //SbVec3f m_accumTranslationWorld;
    // 未设置位移方向：-1，沿x位移：0，沿y位移：1，沿z位移：2
    int m_lastTranslationAxis = -1;
    bool m_hasLastTranslationAxis = false;


    //12.08 新增：
    SbVec3f m_prevDraggerWorldPos;    // 上一帧拖动器原点的世界坐标
    SbVec3f m_stepTranslationDelta;   // 本次帧相对于上一帧的位移（世界坐标）
    SbVec3f m_totalTranslationDelta;  // 从拖动开始到当前的总位移（世界坐标）
    bool m_hasPrevDraggerWorldPos;    // 是否已经有上一帧坐标
    // 累积的总位移（世界坐标）

};

}  // namespace Gui


#endif  // GUI_VIEWPROVIDER_DRAGGER_H