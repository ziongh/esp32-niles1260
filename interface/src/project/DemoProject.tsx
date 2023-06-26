
import React, { FC } from 'react';
import { Navigate, Route, Routes } from 'react-router-dom';

import { Tab } from '@mui/material';

import { RouterTabs, useRouterTab, useLayoutTitle } from '../components';

import DemoInformation from './DemoInformation';
import LightStateRestForm from './LightStateRestForm';
import LightMqttSettingsForm from './LightMqttSettingsForm';
import LightStateWebSocketForm from './LightStateWebSocketForm';

const DemoProject: FC = () => {
  useLayoutTitle("Demo Project");
  const { routerTab } = useRouterTab();

  return (
    <>
      <RouterTabs value={routerTab}>
        <Tab value="socket" label="WebSocket Example" />
      </RouterTabs>
      <Routes>
        <Route path="socket" element={<LightStateWebSocketForm />} />
        <Route path="/*" element={<Navigate replace to="socket" />} />
      </Routes>
    </>
  );
};

export default DemoProject;
