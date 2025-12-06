# Script to generate complete textbook structure
# Run this to create all Parts, Chapters, and lesson placeholders

$ErrorActionPreference = "Stop"

$baseDir = "d:\Sir Zia Native Book Hackathon\physical-ai-humanoid-robotics-textbook\docs"

# Define structure
$structure = @{
    "part-02-ros2-ecosystem" = @{
        title = "Part 2: ROS 2 Ecosystem"
        position = 2
        chapters = @{
            "chapter-03-ros2-architecture" = @{
                title = "Chapter 3: ROS 2 Architecture"
                lessons = @(
                    @{num=1; slug="ros1-to-ros2-evolution"; title="ROS 1 to ROS 2 Evolution"},
                    @{num=2; slug="dds-middleware"; title="DDS Middleware & QoS Policies"},
                    @{num=3; slug="packages-workspaces"; title="Packages and Workspaces"},
                    @{num=4; slug="colcon-build-system"; title="Colcon Build System"}
                )
            }
            "chapter-04-nodes-topics-services" = @{
                title = "Chapter 4: Nodes, Topics, and Services"
                lessons = @(
                    @{num=1; slug="nodes-lifecycle"; title="Nodes and Lifecycle Management"},
                    @{num=2; slug="publishers-subscribers"; title="Publishers and Subscribers"},
                    @{num=3; slug="services-clients"; title="Services and Clients"},
                    @{num=4; slug="custom-messages"; title="Custom Messages and Interfaces"}
                )
            }
            "chapter-05-actionlib-goals" = @{
                title = "Chapter 5: ActionLib and Goal-Based Control"
                lessons = @(
                    @{num=1; slug="actions-vs-services"; title="Actions vs Services"},
                    @{num=2; slug="action-servers"; title="Implementing Action Servers"},
                    @{num=3; slug="action-clients"; title="Action Clients and Feedback"},
                    @{num=4; slug="navigation-actions"; title="Navigation Action Interfaces"}
                )
            }
            "chapter-06-tf2-transformations" = @{
                title = "Chapter 6: TF2 Transformations"
                lessons = @(
                    @{num=1; slug="coordinate-frames"; title="Coordinate Frames and Transforms"},
                    @{num=2; slug="tf2-tree"; title="TF2 Tree Structure"},
                    @{num=3; slug="broadcasting-transforms"; title="Broadcasting Transforms"},
                    @{num=4; slug="listening-transforms"; title="Listening to Transforms"},
                    @{num=5; slug="robot-state-publisher"; title="Robot State Publisher"}
                )
            }
        }
    }
    "part-03-simulation-environments" = @{
        title = "Part 3: Simulation Environments"
        position = 3
        chapters = @{
            "chapter-07-gazebo" = @{
                title = "Chapter 7: Gazebo Classic & Garden"
                lessons = @(
                    @{num=1; slug="gazebo-architecture"; title="Gazebo Architecture"},
                    @{num=2; slug="world-models"; title="World Models and SDF"},
                    @{num=3; slug="sensors-plugins"; title="Sensors and Plugins"},
                    @{num=4; slug="ros2-gazebo-integration"; title="ROS 2 Gazebo Integration"}
                )
            }
            "chapter-08-unity-robotics" = @{
                title = "Chapter 8: Unity Robotics Hub"
                lessons = @(
                    @{num=1; slug="unity-ros2-setup"; title="Unity ROS 2 Setup"},
                    @{num=2; slug="articulation-bodies"; title="Articulation Bodies"},
                    @{num=3; slug="perception-in-unity"; title="Perception in Unity"},
                    @{num=4; slug="ros-unity-communication"; title="ROS-Unity Communication"}
                )
            }
            "chapter-09-urdf-modeling" = @{
                title = "Chapter 9: URDF and Robot Modeling"
                lessons = @(
                    @{num=1; slug="urdf-basics"; title="URDF Basics"},
                    @{num=2; slug="xacro-macros"; title="Xacro Macros"},
                    @{num=3; slug="collision-visual-meshes"; title="Collision and Visual Meshes"},
                    @{num=4; slug="ros2-control-integration"; title="ROS 2 Control Integration"}
                )
            }
        }
    }
    "part-04-nvidia-isaac-platform" = @{
        title = "Part 4: NVIDIA Isaac Platform"
        position = 4
        chapters = @{
            "chapter-10-isaac-sim" = @{
                title = "Chapter 10: Isaac Sim Platform"
                lessons = @(
                    @{num=1; slug="isaac-sim-overview"; title="Isaac Sim Overview"},
                    @{num=2; slug="importing-robots"; title="Importing Robots"},
                    @{num=3; slug="sensors-isaac"; title="Sensors in Isaac Sim"},
                    @{num=4; slug="ros2-bridge-isaac"; title="ROS 2 Bridge"}
                )
            }
            "chapter-11-isaac-ros-perception" = @{
                title = "Chapter 11: Isaac ROS Perception"
                lessons = @(
                    @{num=1; slug="isaac-ros-overview"; title="Isaac ROS Overview"},
                    @{num=2; slug="visual-slam"; title="Visual SLAM"},
                    @{num=3; slug="object-detection-isaac"; title="Object Detection"},
                    @{num=4; slug="depth-estimation"; title="Depth Estimation"}
                )
            }
            "chapter-12-isaac-manipulation" = @{
                title = "Chapter 12: Isaac Manipulation"
                lessons = @(
                    @{num=1; slug="motion-generation"; title="Motion Generation"},
                    @{num=2; slug="grasp-planning"; title="Grasp Planning"},
                    @{num=3; slug="contact-simulation"; title="Contact Simulation"},
                    @{num=4; slug="deformable-objects"; title="Deformable Objects"}
                )
            }
            "chapter-13-isaac-navigation" = @{
                title = "Chapter 13: Isaac Navigation & Planning"
                lessons = @(
                    @{num=1; slug="nvblox-mapping"; title="Nvblox 3D Mapping"},
                    @{num=2; slug="local-path-planning"; title="Local Path Planning"},
                    @{num=3; slug="global-navigation"; title="Global Navigation"},
                    @{num=4; slug="multi-robot-coordination"; title="Multi-Robot Coordination"}
                )
            }
        }
    }
    "part-05-humanoid-development" = @{
        title = "Part 5: Humanoid Development"
        position = 5
        chapters = @{
            "chapter-14-balance-stability" = @{
                title = "Chapter 14: Balance and Stability"
                lessons = @(
                    @{num=1; slug="center-of-mass"; title="Center of Mass"},
                    @{num=2; slug="zero-moment-point"; title="Zero Moment Point"},
                    @{num=3; slug="capture-point"; title="Capture Point"},
                    @{num=4; slug="balance-controllers"; title="Balance Controllers"}
                )
            }
            "chapter-15-inverse-kinematics" = @{
                title = "Chapter 15: Inverse Kinematics"
                lessons = @(
                    @{num=1; slug="forward-kinematics"; title="Forward Kinematics"},
                    @{num=2; slug="jacobian-methods"; title="Jacobian Methods"},
                    @{num=3; slug="analytical-ik"; title="Analytical IK"},
                    @{num=4; slug="numerical-ik"; title="Numerical IK"}
                )
            }
            "chapter-16-whole-body-control" = @{
                title = "Chapter 16: Whole-Body Control"
                lessons = @(
                    @{num=1; slug="task-space-control"; title="Task Space Control"},
                    @{num=2; slug="prioritized-control"; title="Prioritized Control"},
                    @{num=3; slug="contact-constraints"; title="Contact Constraints"},
                    @{num=4; slug="torque-control"; title="Torque Control"}
                )
            }
            "chapter-17-gait-generation" = @{
                title = "Chapter 17: Gait Generation"
                lessons = @(
                    @{num=1; slug="walking-patterns"; title="Walking Patterns"},
                    @{num=2; slug="trajectory-optimization"; title="Trajectory Optimization"},
                    @{num=3; slug="footstep-planning"; title="Footstep Planning"},
                    @{num=4; slug="learning-locomotion"; title="Learning Locomotion"}
                )
            }
        }
    }
    "part-06-conversational-robotics" = @{
        title = "Part 6: Conversational Robotics"
        position = 6
        chapters = @{
            "chapter-18-nlp" = @{
                title = "Chapter 18: Natural Language Processing"
                lessons = @(
                    @{num=1; slug="speech-recognition"; title="Speech Recognition"},
                    @{num=2; slug="intent-detection"; title="Intent Detection"},
                    @{num=3; slug="dialogue-management"; title="Dialogue Management"}
                )
            }
            "chapter-19-vision-language" = @{
                title = "Chapter 19: Vision-Language Models"
                lessons = @(
                    @{num=1; slug="clip-embeddings"; title="CLIP Embeddings"},
                    @{num=2; slug="vlm-grounding"; title="VLM Grounding"},
                    @{num=3; slug="robotics-vlm"; title="Robotics VLM"}
                )
            }
            "chapter-20-gesture-recognition" = @{
                title = "Chapter 20: Gesture Recognition"
                lessons = @(
                    @{num=1; slug="pose-estimation"; title="Pose Estimation"},
                    @{num=2; slug="gesture-classification"; title="Gesture Classification"},
                    @{num=3; slug="gesture-robot-control"; title="Gesture to Robot Control"}
                )
            }
            "chapter-21-real-time-interaction" = @{
                title = "Chapter 21: Real-Time Interaction"
                lessons = @(
                    @{num=1; slug="latency-optimization"; title="Latency Optimization"},
                    @{num=2; slug="multi-modal-fusion"; title="Multi-Modal Fusion"},
                    @{num=3; slug="social-navigation"; title="Social Navigation"}
                )
            }
        }
    }
    "part-07-capstone-project" = @{
        title = "Part 7: Capstone Project"
        position = 7
        chapters = @{
            "chapter-22-capstone" = @{
                title = "Chapter 22: Building Your Humanoid System"
                lessons = @(
                    @{num=1; slug="project-planning"; title="Project Planning"},
                    @{num=2; slug="system-architecture"; title="System Architecture"},
                    @{num=3; slug="implementation-strategy"; title="Implementation Strategy"},
                    @{num=4; slug="integration-testing"; title="Integration Testing"},
                    @{num=5; slug="optimization"; title="Optimization"},
                    @{num=6; slug="documentation"; title="Documentation"},
                    @{num=7; slug="presentation"; title="Presentation"},
                    @{num=8; slug="deployment"; title="Deployment"}
                )
            }
        }
    }
}

Write-Host "Generating complete textbook structure..." -ForegroundColor Green
Write-Host "Total: 7 Parts, 19 Chapters, 70+ lessons" -ForegroundColor Cyan
Write-Host ""

$totalFiles = 0

foreach ($partKey in $structure.Keys | Sort-Object) {
    $part = $structure[$partKey]
    $partDir = Join-Path $baseDir $partKey
    
    Write-Host "Creating $($part.title)..." -ForegroundColor Yellow
    
    # Create part directory
    New-Item -ItemType Directory -Path $partDir -Force | Out-Null
    
    # Create part _category_.json
    $partCategory = @"
{
  "label": "$($part.title)",
  "position": $($part.position),
  "link": {
    "type": "generated-index",
    "description": "Learn the fundamentals and advanced techniques.",
    "slug": "/$partKey"
  },
  "collapsed": false
}
"@
    $partCategory | Out-File -FilePath (Join-Path $partDir "_category_.json") -Encoding utf8
    $totalFiles++
    
    foreach ($chapterKey in $part.chapters.Keys | Sort-Object) {
        $chapter = $part.chapters[$chapterKey]
        $chapterDir = Join-Path $partDir $chapterKey
        
        Write-Host "  - $($chapter.title) ($($chapter.lessons.Count) lessons)" -ForegroundColor Gray
        
        # Create chapter directory
        New-Item -ItemType Directory -Path $chapterDir -Force | Out-Null
        
        # Create chapter _category_.json
        $chapterCategory = @"
{
  "label": "$($chapter.title)",
  "position": $($chapterKey -replace '.*-(\d+)-.*', '$1'),
  "link": {
    "type": "doc",
    "id": "$partKey/$chapterKey/index"
  },
  "collapsed": false
}
"@
        $chapterCategory | Out-File -FilePath (Join-Path $chapterDir "_category_.json") -Encoding utf8
        $totalFiles++
        
        # Create chapter index.md placeholder
        $chapterIndex = @"
---
sidebar_position: 0
---

# $($chapter.title)

:::info Chapter Under Development
This chapter is currently being developed. Lessons will be added progressively.

**Estimated Completion**: Coming soon
:::

## Chapter Overview

This chapter will cover essential topics in $($chapter.title.ToLower()).

## Lessons

"@
        foreach ($lesson in $chapter.lessons) {
            $chapterIndex += "`n### $($lesson.num). [$($lesson.title)](./$($lesson.num.ToString('D2'))-$($lesson.slug).md)`n"
            $chapterIndex += "Coming soon`n"
        }
        
        $chapterIndex | Out-File -FilePath (Join-Path $chapterDir "index.md") -Encoding utf8
        $totalFiles++
        
        # Create lesson placeholders
        foreach ($lesson in $chapter.lessons) {
            $lessonFile = "$($lesson.num.ToString('D2'))-$($lesson.slug).md"
            $lessonPath = Join-Path $chapterDir $lessonFile
            
            $lessonContent = @"
---
sidebar_position: $($lesson.num)
title: "$($lesson.title)"
description: "Learn about $($lesson.title.ToLower()) in depth."
---

# $($lesson.title)

:::info Coming Soon
This lesson is currently under development. Check back soon for comprehensive content.

**Expected Completion**: This lesson will be available soon.
:::

## Learning Objectives

By the end of this lesson, you will be able to:

1. Understand the fundamentals of $($lesson.title.ToLower())
2. Apply concepts in practical robotics scenarios
3. Implement solutions using ROS 2 and related tools

## Further Reading

While this lesson is being developed, explore related documentation and resources.

## What's Next?

Continue with the next lesson in this chapter.

---

*This lesson is part of $($chapter.title)*
"@
            $lessonContent | Out-File -FilePath $lessonPath -Encoding utf8
            $totalFiles++
        }
    }
    
    Write-Host ""
}

Write-Host "Structure generation complete!" -ForegroundColor Green
Write-Host "Total files created: $totalFiles" -ForegroundColor Cyan
Write-Host ""
Write-Host "Next steps:" -ForegroundColor Yellow
Write-Host "1. Run npm run build to verify structure" -ForegroundColor White
Write-Host "2. Generate individual lessons using the lesson generation prompt" -ForegroundColor White
Write-Host "3. Review generated content in Docusaurus" -ForegroundColor White
