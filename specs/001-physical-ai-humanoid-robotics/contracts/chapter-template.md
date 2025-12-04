# Chapter Template Contract

**Purpose**: Standardized structure for all book chapters to ensure consistency per Constitution Principle III (Consistency).

---

## Chapter Structure

Every chapter MUST follow this structure:

```markdown
---
id: {module-id}-{chapter-number}
title: "{Chapter Title}"
sidebar_label: "{Short Label}"
sidebar_position: {1-5}
description: "{One-line SEO description}"
keywords: [{keyword1}, {keyword2}, {keyword3}]
---

# {Chapter Title}

## Learning Objectives

By the end of this chapter, you will be able to:

- [ ] {Objective 1 - actionable verb + outcome}
- [ ] {Objective 2 - actionable verb + outcome}
- [ ] {Objective 3 - actionable verb + outcome}

## Prerequisites

- {Prerequisite 1 with link to relevant chapter}
- {Prerequisite 2}

---

## {Section 1 Title}

{Content with concepts explained before advanced topics}

### {Subsection if needed}

{More detailed content}

:::info Version Note
This chapter uses **{Tool} {Version}** on **{OS}**.
:::

---

## {Section 2 Title}

{Content}

### Code Example: {Descriptive Title}

```{language}
# File: {path/to/file.ext}
{code}
```

**Expected Output:**
```
{expected output}
```

---

## Hands-On Exercise

**Objective:** {What reader will build/accomplish}

**Estimated Time:** {X} minutes

### Steps

1. {Step 1 with specific action}
2. {Step 2}
3. {Step 3}

### Verification

{How to confirm success - specific command or observable outcome}

---

## Summary

In this chapter, you learned:

- {Key takeaway 1}
- {Key takeaway 2}
- {Key takeaway 3}

## Next Steps

Continue to [{Next Chapter Title}](./{next-chapter-slug}.md) to learn {what comes next}.

---

## References

- [{Source Title}]({URL})
- [{Source Title}]({URL})
```

---

## Required Elements

| Element | Required | Location | Constitution Principle |
|---------|----------|----------|----------------------|
| Frontmatter | Yes | Top | Modularity (IV) |
| Learning Objectives | Yes | After title | Clarity (II) |
| Prerequisites | Yes | After objectives | Clarity (II) |
| Version Note | Yes | First section | Version Awareness (VII) |
| Code Examples | Module-dependent | Body | Practicality (VI) |
| Hands-On Exercise | Recommended | Before summary | Practicality (VI) |
| Summary | Yes | End | Clarity (II) |
| References | Yes | End | Accuracy (I) |

---

## Callout Styles

Use Docusaurus admonitions consistently:

```markdown
:::note
General information or clarification.
:::

:::tip
Best practice or helpful suggestion.
:::

:::info
Version-specific or configuration note.
:::

:::caution
Potential issue or thing to watch out for.
:::

:::danger
Critical warning - may cause errors or data loss.
:::
```

---

## Code Block Requirements

1. **Language Tag**: Always specify (python, bash, yaml, xml, json)
2. **File Path**: Include as comment on first line
3. **Tested**: All code must run without errors
4. **Version**: Note ROS 2 version if applicable

```python
# File: src/my_package/my_node.py
# ROS 2: Jazzy

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node started')

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Word Count Guidelines

| Module | Min Words | Max Words | Target |
|--------|-----------|-----------|--------|
| 1 (ROS 2) | 800 | 1,200 | 1,000 |
| 2 (Simulation) | 1,000 | 1,500 | 1,250 |
| 3 (Isaac) | 1,000 | 1,500 | 1,250 |
| 4 (VLA) | 1,200 | 1,800 | 1,500 |

---

## Validation Checklist

Before marking a chapter complete:

- [ ] Frontmatter is valid YAML
- [ ] Learning objectives are specific and measurable
- [ ] All code examples run without errors
- [ ] Expected outputs are accurate
- [ ] All external links resolve
- [ ] Version numbers are explicit
- [ ] Word count is within range
- [ ] Callouts use correct syntax
- [ ] References use Markdown link format
- [ ] No [NEEDS CLARIFICATION] markers remain
