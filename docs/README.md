# docs

`src/`와 같은 기준으로 역할별로 나눈다. 두 역할 이상에 해당하면 `common/`에 둔다.

```
docs/
├── common/              모두가 보는 문서
│   ├── setup.md             개발 환경 세팅 (.bashrc, direnv, venv, 빌드)
│   ├── navigation.md        localization → rviz → nav2 띄우기, 두 로봇 공용 지도
│   └── requirements.md      노션 BR/SR 요약, 수치 기준, 기능→패키지 매핑
├── follow/              자유 관람 팀 (src/arti_follow/)
│   ├── README.md            범위, 노드·인터페이스·파라미터 설계, 확인 필요 사항
│   ├── run_and_test_guide.md  실제/부분 시나리오, 단품/bag/모듈 단위 빌드·실행 가이드
│   └── tasks.md             노션 작업 목록 T01~T46 체크리스트
├── course/              코스 관람 (src/arti_pc2_nav/, robot4)
│   ├── README.md            다른 팀과 주고받는 것, 패키지 README 링크
│   └── arti_course_flow.drawio  흐름 그림 (예전 구조 기준)
└── troubleshooting/     에러 기록 (하위 폴더도 env/robot/common/follow)
```

## 규칙

- 역할 문서(`follow/` 등)는 그 팀이 관리한다. 설계가 바뀌면 코드와 같은 PR에서 문서도 고친다.
- 다른 팀과 주고받는 토픽·서비스·액션이 정해지면 역할 문서의 **"다른 팀과 주고받는 것"** 표에 적는다. 실제 정의는 `src/common/arti_interfaces`에 둔다.
- 문서가 커지면 역할 폴더 안에 파일을 더 만든다 (예: `follow/reid.md`). 새 파일을 만들면 그 폴더 README에 링크를 건다.
- 원본 기획은 노션이다. 여기에는 **코드를 짜는 데 필요한 내용**만 둔다.
