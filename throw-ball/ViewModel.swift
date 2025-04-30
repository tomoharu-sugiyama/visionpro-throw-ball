import ARKit
import RealityKit
import SwiftUI

extension SIMD4 {
    var xyz: SIMD3<Scalar> {
        self[SIMD3(0, 1, 2)]
    }
}

@Observable
@MainActor
class ViewModel {
    let session = ARKitSession()
    let handTracking = HandTrackingProvider()
    let sceneReconstruction = SceneReconstructionProvider()
    let worldTracking = WorldTrackingProvider()
    
    private var meshEntities = [UUID: ModelEntity]()
    var contentEntity = Entity()
    var latestHandTracking: HandsUpdates = .init(left: nil, right: nil)
    var leftHandEntity = Entity()
    var rightHandEntity = Entity()
    var gripFrameCount: Int = 0 // フレームカウント用変数を追加
    var isGlab: Bool = false
    
    var gripPosition: SIMD3<Float>? = nil  // 握った瞬間の位置
    var releasePosition: SIMD3<Float>? = nil  // 離した瞬間の位置
    
    var initialVelocity: SIMD3<Float>? = nil
    var velocityStartTime: TimeInterval? = nil
    let decelerationDuration: TimeInterval = 1.0 // 減速にかける秒数

    
    enum OperationLock {
        case none
        case right
        case left
    }
    
    enum HandGlab {
        case right
        case left
    }
    
    var entitiyOperationLock = OperationLock.none
    
    // ここで反発係数を決定している可能性あり
    let material = PhysicsMaterialResource.generate(friction: 0.1,restitution: 1.0)
    
    struct HandsUpdates {
        var left: HandAnchor?
        var right: HandAnchor?
    }
    
    var errorState = false
    
    func setupContentEntity() -> Entity {
        return contentEntity
    }
    

    
    var dataProvidersAreSupported: Bool {
        HandTrackingProvider.isSupported && SceneReconstructionProvider.isSupported
    }
    
    var isReadyToRun: Bool {
        handTracking.state == .initialized && sceneReconstruction.state == .initialized
    }
    
    func processReconstructionUpdates() async {
        for await update in sceneReconstruction.anchorUpdates {
            let meshAnchor = update.anchor
            
            guard let shape = try? await ShapeResource.generateStaticMesh(from: meshAnchor) else { continue }
            switch update.event {
            case .added:
                let entity = ModelEntity()
                entity.transform = Transform(matrix: meshAnchor.originFromAnchorTransform)
                entity.collision = CollisionComponent(shapes: [shape], isStatic: true)
                entity.components.set(InputTargetComponent())
                
                // mode が dynamic でないと物理演算が適用されない
                entity.physicsBody = PhysicsBodyComponent(mode: .dynamic)
                
                meshEntities[meshAnchor.id] = entity
                contentEntity.addChild(entity)
            case .updated:
                guard let entity = meshEntities[meshAnchor.id] else { continue }
                entity.transform = Transform(matrix: meshAnchor.originFromAnchorTransform)
                entity.collision?.shapes = [shape]
            case .removed:
                meshEntities[meshAnchor.id]?.removeFromParent()
                meshEntities.removeValue(forKey: meshAnchor.id)
            }
        }
    }
    
    func monitorSessionEvents() async {
        for await event in session.events {
            switch event {
            case .authorizationChanged(type: _, status: let status):
                print("Authorization changed to: \(status)")
                
                if status == .denied {
                    errorState = true
                }
            case .dataProviderStateChanged(dataProviders: let providers, newState: let state, error: let error):
                print("Data provider changed: \(providers), \(state)")
                if let error {
                    print("Data provider reached an error state: \(error)")
                    errorState = true
                }
            @unknown default:
                fatalError("Unhandled new event type \(event)")
            }
        }
    }
    
    func processHandUpdates() async {
        for await update in handTracking.anchorUpdates {
            switch update.event {
            case .updated:
                let anchor = update.anchor
                
                guard anchor.isTracked else { continue }
                
                if anchor.chirality == .left {
                    latestHandTracking.left = anchor
                    guard let handAnchor = latestHandTracking.left else { continue }
                    glabGesture(handAnchor: handAnchor,handGlab: .left)
                } else if anchor.chirality == .right {
                    latestHandTracking.right = anchor
                    guard let handAnchor = latestHandTracking.right else { continue }
                    glabGesture(handAnchor: handAnchor,handGlab: .right)
                }
            default:
                break
            }
        }
    }
    
    
    
    // ボールの初期化
    func initBall() {
        guard let originTransform = latestHandTracking.right?.originFromAnchorTransform else { return }
        guard let handSkeletonAnchorTransform =  latestHandTracking.right?.handSkeleton?.joint(.indexFingerTip).anchorFromJointTransform else { return }
        
        let originFromIndex = originTransform * handSkeletonAnchorTransform
        let place = originFromIndex.columns.3.xyz
        //ボールの生成
        let ball = ModelEntity(
            mesh: .generateSphere(radius: 0.05),
            materials: [SimpleMaterial(color: .white, isMetallic: true)],
            collisionShape: .generateSphere(radius: 0.05),
            mass: 1.0
        )
        ball.name = "ball"
        ball.setPosition(place, relativeTo: nil)
        ball.components.set(InputTargetComponent(allowedInputTypes: .all))
        //ボールの重力切るやつ
        ball.physicsBody?.isAffectedByGravity = false
        //ボールを徐々に遅くする
        ball.physicsBody?.linearDamping = 1.0
        // mode が dynamic でないと物理演算が適用されない
        ball.components.set(PhysicsBodyComponent(
            shapes: [ShapeResource.generateSphere(radius: 0.05)],
            mass: 1.0,
            material: material,
            mode: .dynamic
        ))
        //ボールの重力切るやつ
        ball.physicsBody?.isAffectedByGravity = false
  
        contentEntity.addChild(ball)
    }
    var gripStartPosition: SIMD3<Float>? = nil // 握り始めの位置
    var lastPosition: SIMD3<Float>? = nil // 最後の位置
    // 握るジェスチャーの検出
    func glabGesture(handAnchor: HandAnchor, handGlab: HandGlab) {
        if(handGlab == .right && entitiyOperationLock == .left || handGlab == .left && entitiyOperationLock == .right) {
            return
        }
        
        guard let wrist = handAnchor.handSkeleton?.joint(.wrist).anchorFromJointTransform else { return }
        guard let thumbIntermediateTip = handAnchor.handSkeleton?.joint(.thumbIntermediateTip).anchorFromJointTransform else { return }
        guard let indexFingerTip = handAnchor.handSkeleton?.joint(.indexFingerTip).anchorFromJointTransform else { return }
        guard let middleFingerTip = handAnchor.handSkeleton?.joint(.middleFingerTip).anchorFromJointTransform else { return }
        guard let ringFingerTip = handAnchor.handSkeleton?.joint(.ringFingerTip).anchorFromJointTransform else { return }
        guard let littleFingerTip = handAnchor.handSkeleton?.joint(.littleFingerTip).anchorFromJointTransform else { return }
        //ボール掴んだ瞬間と離した瞬間の手の位置の基準
        guard let middleFingerMetacarpal = handAnchor.handSkeleton?.joint(.middleFingerMetacarpal).anchorFromJointTransform else { return }
        
        let thumbIntermediateTipToWristDistance = simd_length_squared(wrist.columns.3.xyz - thumbIntermediateTip.columns.3.xyz)
        let indexFingerTipToWristDistance = simd_length_squared(wrist.columns.3.xyz - indexFingerTip.columns.3.xyz)
        let middleFingerTipToWristDistance = simd_length_squared(wrist.columns.3.xyz - middleFingerTip.columns.3.xyz)
        let ringFingerTipToWristDistance = simd_length_squared(wrist.columns.3.xyz - ringFingerTip.columns.3.xyz)
        let littleFingerTipToWristDistance = simd_length_squared(wrist.columns.3.xyz - littleFingerTip.columns.3.xyz)
        
        // ボールエンティティの取得
        guard let ballEntity = contentEntity.children.first(where: { $0.name == "ball" }) as? ModelEntity else { return }
        
        // ボールとの距離を計算
        let ballPositionTransformMatrix = contentEntity.transform.matrix * ballEntity.transform.matrix
        let handPositionTransformMatrix = handAnchor.originFromAnchorTransform * indexFingerTip
        let ballHandLength = simd_length_squared(ballPositionTransformMatrix.columns.3.xyz - handPositionTransformMatrix.columns.3.xyz)
        
        if  ballHandLength > 0.20 {
            isGlab = false
            return
        }
        
        // 手の形を判定
        if thumbIntermediateTipToWristDistance > 0.01
            && indexFingerTipToWristDistance > 0.01
            && middleFingerTipToWristDistance > 0.01
            && ringFingerTipToWristDistance > 0.01
            && littleFingerTipToWristDistance > 0.01 {
            // 離した瞬間の処理（1回だけ）
            if isGlab {
                //ての基準の位置の取得
                if let middleFingerMetacarpal = handAnchor.handSkeleton?.joint(.middleFingerMetacarpal).anchorFromJointTransform {
                    let worldTransform = handAnchor.originFromAnchorTransform * middleFingerMetacarpal
                    let position = worldTransform.columns.3.xyz
                    print("離した瞬間の middleFingerMetacarpal の位置: \(position)")
                    releasePosition = position
                    
                    // ベクトルの強さと方向を計算
                    if let grip = gripPosition {
                        let delta = position - grip
                        //ベクトルの方向
                        let direction = simd_normalize(delta)
                        //ベクトルの強さ
                        let strength = simd_length(delta)
                        //飛ばす方向と力
                        let force = -direction * strength * 800.0
                        //力を加えるときの初速を保存
                        initialVelocity = force
                        velocityStartTime = Date().timeIntervalSinceReferenceDate
                        
                        print("ベクトルの方向: \(direction)")
                        print("ベクトルの強さ: \(strength)")
                        print("加える強さ: \(force)")
                        //ボールに力を加える
                        ballEntity.addForce(force, relativeTo: nil)
                    }
                    // リセット
                    gripPosition = nil
                    releasePosition = nil
                }
            }
            
            // 物理演算を再開
            ballEntity.components.set((PhysicsBodyComponent(shapes: [ShapeResource.generateSphere(radius: 0.05)], mass: 1.0, material: material, mode: .dynamic)))
            //ボールを徐々に遅くする
            ballEntity.physicsBody?.linearDamping = 1.0
            //重力を切る
            ballEntity.physicsBody?.isAffectedByGravity = false
            
            isGlab = false
            entitiyOperationLock = .none
            return
        }
        
        //print(Date().timeIntervalSince1970,"\tにぎる")
        
        // 握っている間は物理演算を解除
        ballEntity.components.set((PhysicsBodyComponent(shapes: [ShapeResource.generateSphere(radius: 0.05)], mass: 1.0, material: material, mode: .static)))
        //重力を切る
        ballEntity.physicsBody?.isAffectedByGravity = false
        // 握った瞬間だけ実行（isGlabがfalseだった→握ったと判定されたとき）
        if !isGlab {
            // middleFingerMetacarpalのワールド座標を取得して表示
            let middleFingerMetacarpalWorldTransform = handAnchor.originFromAnchorTransform * middleFingerMetacarpal
            let position = middleFingerMetacarpalWorldTransform.columns.3.xyz
            gripPosition = position
            print("握った瞬間の middleFingerMetacarpal の位置: \(position)")
        }
        
        ballEntity.transform = Transform(
            matrix: matrix_multiply(handAnchor.originFromAnchorTransform, (handAnchor.handSkeleton?.joint(.indexFingerTip).anchorFromJointTransform)!)
        )
        
        isGlab = true
        
        // 手の向きに力を加える
        //        ballEntity.addForce(calculateForceDirection(handAnchor: handAnchor) * 4, relativeTo: nil)
        entitiyOperationLock = handGlab == .right ? .right : .left
    }
//    //ボールを減速させる
//    func updateBallVelocityPerFrame() {
//        guard let velocityStartTime, let initialVelocity else { return }
//        guard let ballEntity = contentEntity.children.first(where: { $0.name == "ball" }) as? ModelEntity else { return }
//
//        let currentTime = Date().timeIntervalSinceReferenceDate
//        let elapsedTime = currentTime - velocityStartTime
//        let t = min(elapsedTime / decelerationDuration, 1.0)
//
//        // 二次関数的に減速
//        let currentVelocity = initialVelocity * Float(pow(1 - t, 2))
//        ballEntity.physicsMotion?.linearVelocity = currentVelocity
//
//        if t >= 1.0 {
//            self.initialVelocity = nil
//            self.velocityStartTime = nil
//            ballEntity.physicsMotion?.linearVelocity = .zero
//        }
//    }
    
    func simd_distance(_ a: SIMD3<Float>, _ b: SIMD3<Float>) -> Float {
        return simd_length(a - b)
    }
}

