// Inspector Gadgets // Copyright 2019 Kybernetik //

#if UNITY_2017_1_OR_NEWER

using UnityEngine;

namespace InspectorGadgets
{
    /// <summary>
    /// Holds a text comment for a <see cref="GameObject"/> which can be viewed and edited in the inspector.
    /// <para></para>
    /// By default, this script sets itself to be excluded from the build.
    /// </summary>
    [HelpURL(Strings.APIDocumentationURL + "/CommentComponent")]
    public sealed class CommentComponent : MonoBehaviour, IComment
    {
        /************************************************************************************************************************/

        [SerializeField, TextArea]
        private string _Text;

        /// <summary>The text of this comment.</summary>
        public string Text
        {
            get { return _Text; }
            set { _Text = value; }
        }

        /************************************************************************************************************************/

        /// <summary>False if this script is set to <see cref="HideFlags.DontSaveInBuild"/>.</summary>
        public bool IncludeInBuild
        {
            get
            {
                return (hideFlags &= HideFlags.DontSaveInBuild) == 0;
            }
            set
            {
                if (value)
                    hideFlags &= ~HideFlags.DontSaveInBuild;
                else
                    hideFlags |= HideFlags.DontSaveInBuild;
            }
        }

        /************************************************************************************************************************/

        private void Reset()
        {
            IncludeInBuild = false;
        }

        /************************************************************************************************************************/
    }
}

/************************************************************************************************************************/

#if UNITY_EDITOR
namespace InspectorGadgets.Editor
{
    [UnityEditor.CustomEditor(typeof(CommentComponent))]
    internal sealed class CommentComponentEditor : CommentEditor { }
}
#endif

#endif
