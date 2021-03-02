// Inspector Gadgets // Copyright 2019 Kybernetik //

#pragma warning disable CS0618 // Type or member is obsolete: EditorApplication.playmodeStateChanged

#if UNITY_EDITOR

using UnityEditor;
using UnityEngine;

namespace InspectorGadgets.Editor
{
    [HelpURL(Strings.APIDocumentationURL + ".Editor/BlendShapeViewer")]
    internal sealed class BlendShapeViewer : MonoBehaviour
    {
        /************************************************************************************************************************/

        private void OnValidate()
        {
            hideFlags |= HideFlags.DontSave;
        }

        /************************************************************************************************************************/
    }

    /************************************************************************************************************************/

    [InitializeOnLoad]
    [CustomEditor(typeof(BlendShapeViewer)), CanEditMultipleObjects]
    internal sealed class BlendShapeViewerEditor : UnityEditor.Editor
    {
        /************************************************************************************************************************/

        private static readonly AutoPrefs.EditorBool
            IsEnabled = new AutoPrefs.EditorBool(Strings.PrefsKeyPrefix + "BlendShapeViewerEditor", true);

        private static BlendShapeViewer _Instance;

        private SkinnedMeshRenderer _TargetRenderer;
        private SerializedObject _TargetSerializedObject;
        private SerializedProperty[] _BlendShapeWeights;

        /************************************************************************************************************************/

        static BlendShapeViewerEditor()
        {
            EditorApplication.delayCall += () =>
            {
                if (IsEnabled)
                {
                    Selection.selectionChanged += OnSelectionChanged;
                    EditorApplication.playmodeStateChanged += OnSelectionChanged;
                }
            };
        }

        /************************************************************************************************************************/

        public static void Enable()
        {
            Selection.selectionChanged += OnSelectionChanged;
            EditorApplication.playmodeStateChanged += OnSelectionChanged;
            OnSelectionChanged();
        }

        /************************************************************************************************************************/

        private static bool ShouldShow()
        {
            var activeGameObject = Selection.activeGameObject;
            if (activeGameObject == null ||
                !activeGameObject.activeInHierarchy ||
                EditorUtility.IsPersistent(activeGameObject))
                return false;

            var renderer = activeGameObject.GetComponent<SkinnedMeshRenderer>();
            if (renderer == null ||
                renderer.sharedMesh == null ||
                renderer.sharedMesh.blendShapeCount == 0)
                return false;

            return true;
        }

        /************************************************************************************************************************/

        private static void OnSelectionChanged()
        {
            if (_Instance == null && ShouldShow())
            {
                var activeGameObject = Selection.activeGameObject;
                _Instance = activeGameObject.GetComponent<BlendShapeViewer>();
                if (_Instance == null)
                    _Instance = activeGameObject.AddComponent<BlendShapeViewer>();
                _Instance.hideFlags |= HideFlags.DontSave;
            }
        }

        /************************************************************************************************************************/

        private void OnEnable()
        {
            if (target == null)
                return;

            target.hideFlags |= HideFlags.DontSave;

            _TargetRenderer = ((Component)target).gameObject.GetComponent<SkinnedMeshRenderer>();
            if (_TargetRenderer == null)
            {
                DestroyImmediate(target);
                return;
            }

            _TargetSerializedObject = new SerializedObject(_TargetRenderer);
            var property = _TargetSerializedObject.FindProperty("m_BlendShapeWeights");

            var blendShapeCount = property.arraySize;
            _BlendShapeWeights = new SerializedProperty[blendShapeCount];

            var i = 0;

            property.Next(true);
            property.Next(true);
            while (property.Next(true))
            {
                _BlendShapeWeights[i++] = property.Copy();
                if (i >= blendShapeCount)
                    break;
            }
        }

        /************************************************************************************************************************/

        public override void OnInspectorGUI()
        {
            if (_TargetRenderer == null)
            {
                DestroyImmediate(target);
                return;
            }

            var mesh = _TargetRenderer.sharedMesh;
            if (mesh == null)
                return;

            _TargetSerializedObject.Update();

            for (int i = 0; i < _BlendShapeWeights.Length; i++)
            {
                var weight = _BlendShapeWeights[i];
                var label = mesh.GetBlendShapeName(i);
                weight.floatValue = EditorGUILayout.Slider(label, weight.floatValue, 0, 100);
            }

            _TargetSerializedObject.ApplyModifiedProperties();
        }

        /************************************************************************************************************************/

        private void OnDisable()
        {
            DestroyImmediate(target);
        }

        /************************************************************************************************************************/
    }

    /************************************************************************************************************************/
}

#endif
