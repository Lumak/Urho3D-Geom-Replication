//
// Copyright (c) 2008-2016 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#pragma once

#include "Sample.h"

namespace Urho3D
{
class Node;
class Scene;
class Text3D;
}

//=============================================================================
//=============================================================================
struct PRotScale
{
    Vector3     pos;
    Quaternion  rot;
    float       scale;
};

class GeomReplicator : public StaticModel
{
    URHO3D_OBJECT(GeomReplicator, StaticModel);

    struct MoveAccumulator
    {
        MoveAccumulator() 
            : origPos(Vector3::ZERO), deltaMovement(Vector3::ZERO), reversing(false)
        {
            timeAccumlated = Random() * 0.2f;
        }

        Vector3 origPos;
        Vector3 deltaMovement;
        Timer   timer;
        float   timeAccumlated;
        bool    reversing;
    };

public:
    static void RegisterObject(Context* context)
    {
        context->RegisterFactory<GeomReplicator>();
    }

    GeomReplicator(Context *context) 
        : StaticModel(context), numVertsPerGeom(0), batchCount_(0), showGeomVertIndeces_(false)
    {
    }

    virtual ~GeomReplicator()
    {
    }

    unsigned Replicate(const PODVector<PRotScale> &qplist, const Vector3 &normalOverride=Vector3::ZERO);
    bool ConfigWindVelocity(const PODVector<unsigned> &vertIndecesToMove, unsigned batchCount, 
                            const Vector3 &velocity, float cycleTimer);
    void WindAnimationEnabled(bool enable);
    void ShowGeomVertIndeces(bool show);

protected:
    unsigned ReplicateIndeces(IndexBuffer *idxbuffer, unsigned numVertices, unsigned expandSize);
    void AnimateVerts();
    void RenderGeomVertIndeces();
    void HandleUpdate(StringHash eventType, VariantMap& eventData);

protected:
    PODVector<MoveAccumulator>  animatedVertexList_;
    PODVector<unsigned>         vertIndecesToMove_;

    unsigned                    numVertsPerGeom;
    unsigned                    batchCount_;
    unsigned                    currentVertexIdx_;
    Timer                       timerUpdate_;
    Vector3                     windVelocity_;
    float                       cycleTimer_;
    float                       timeStepAccum_;

    // dbg
    Vector<Node*>               nodeText3DVertList_;
    bool                        showGeomVertIndeces_;

protected:
    enum FrameRateType { FrameRate_MSec = 32    };
    enum MaxTimeType   { MaxTime_Elapsed = 1000 };
};

//=============================================================================
//=============================================================================
class StaticScene : public Sample
{
    URHO3D_OBJECT(StaticScene, Sample);

public:
    StaticScene(Context* context);

    virtual void Setup();
    virtual void Start();

protected:
    void CreateScene();
    void CreateStatusText();
    void SetupViewport();
    void MoveCamera(float timeStep);
    void SubscribeToEvents();
    void HandleUpdate(StringHash eventType, VariantMap& eventData);

protected:
    WeakPtr<Text> textStatus_;

    Timer         fpsTimer_;
    int           framesCount_;

    PODVector<PRotScale> qpList_;

    unsigned      timeToLoad_;
    Timer         keyDebounceTimer_;

    // replicator
    SharedPtr<GeomReplicator> vegReplicator_;
    WeakPtr<Node> nodeRep_;
};
