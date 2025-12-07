import React from 'react';
import SidebarToggleButton from '../../../../components/SidebarToggleButton';

// It's generally not recommended to swizzle internal components, but this is necessary for the desired UI/UX.
// We are assuming the props are passed correctly from the parent component.
// @ts-ignore
import type {Props} from '@theme/DocSidebar/Desktop/CollapseButton';

export default function CollapseButton({onClick, isCollapsed}: Props) {
  return (
    <SidebarToggleButton onClick={onClick} isCollapsed={isCollapsed} />
  );
}
